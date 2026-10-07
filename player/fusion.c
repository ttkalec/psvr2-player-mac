/*
 * Head-orientation fusion. The gyro (2000 Hz) is integrated continuously;
 * each SLAM pose (~63 Hz, ~20-35 ms old on arrival) is compared with the
 * integrated orientation at the SLAM timestamp. Part of the difference is
 * applied at once, and the difference also trains a gyro bias estimate.
 *
 * Replacing the integration base with every SLAM pose instead made the view
 * jump at 63 Hz by gyro bias x time since the pose (the PS VR2 gyro reads
 * ~3.3 deg/s at rest: 0.04-0.1 deg per jump), plus gyro scale error during
 * head turns.
 */
#include "fusion.h"

#include <math.h>
#include <string.h>

/* Share of the SLAM difference applied per pose (~63 Hz): about a 23 ms
 * time constant. Slower blending is smoother but lags a gyro with a scale
 * error; this keeps a 2% scale error within ~15% of restarting at each pose
 * while cutting frame-to-frame jitter 2.5-7x (tools/test-fusion). */
#define FUSION_KP 0.5f
/* Bias learning per pose: a ~2 s time constant (KP / KI seconds). Learning
 * at every speed tolerated a +-0.4 ms IMU/SLAM clock offset best. */
#define FUSION_KI (FUSION_KP * 0.5f)
/* Larger differences (startup, tracking relocalization) are applied whole. */
#define FUSION_SNAP_RAD (3.0f * (float)M_PI / 180.0f)
#define FUSION_BIAS_LIMIT (10.0f * (float)M_PI / 180.0f)

static void quat_mul(const float a[4], const float b[4], float out[4])
{
	float w = a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3];
	float x = a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2];
	float y = a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1];
	float z = a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0];
	out[0] = w;
	out[1] = x;
	out[2] = y;
	out[3] = z;
}

static void quat_normalize(float q[4])
{
	float len = sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
	if (len > 1e-6f) {
		for (int i = 0; i < 4; i++) {
			q[i] /= len;
		}
	}
}

/* Rotation by the vector v (axis * angle, radians). */
static void quat_exp(const float v[3], float out[4])
{
	float angle = sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
	float s = angle < 1e-9f ? 0.5f : sinf(angle * 0.5f) / angle;
	out[0] = cosf(angle * 0.5f);
	out[1] = v[0] * s;
	out[2] = v[1] * s;
	out[3] = v[2] * s;
}

/* Rotation vector of a unit quaternion, along the shorter way round. */
static void quat_log(const float q[4], float out[3])
{
	float sign = q[0] < 0.0f ? -1.0f : 1.0f;
	float w = q[0] * sign;
	float len = sqrtf(q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
	float scale = len < 1e-9f ? 2.0f : 2.0f * atan2f(len, w) / len;
	for (int i = 0; i < 3; i++) {
		out[i] = q[i + 1] * sign * scale;
	}
}

/* v rotated by the inverse of q. */
static void rotate_inverse(const float q[4], const float v[3], float out[3])
{
	float conj[4] = {q[0], -q[1], -q[2], -q[3]};
	float p[4] = {0, v[0], v[1], v[2]};
	float t[4], r[4];
	quat_mul(conj, p, t);
	quat_mul(t, q, r);
	out[0] = r[1];
	out[1] = r[2];
	out[2] = r[3];
}

void fusion_reset(struct fusion_state *f)
{
	memset(f, 0, sizeof(*f));
	f->q[0] = 1.0f;
}

void fusion_imu(struct fusion_state *f, uint32_t vts_us, const float gyro[3], float dt)
{
	float step[3], dq[4];
	for (int i = 0; i < 3; i++) {
		f->rate[i] = gyro[i] - f->bias[i];
		step[i] = f->rate[i] * dt;
	}
	quat_exp(step, dq);
	quat_mul(f->q, dq, f->q);
	quat_normalize(f->q);

	/* The headset stamps IMU samples in pairs with one 1 ms VTS value;
	 * the second sample of a pair is 0.5 ms later. */
	f->pair_index = (vts_us == f->last_vts_us && f->count > 0) ? f->pair_index + 1 : 0;
	f->last_vts_us = vts_us;
	struct fusion_sample *s = &f->history[f->head];
	s->time_us = vts_us + (uint32_t)(f->pair_index * dt * 1e6f + dt * 0.5e6f);
	memcpy(s->q, f->q, sizeof(f->q));
	f->head = (f->head + 1) % FUSION_HISTORY;
	if (f->count < FUSION_HISTORY) {
		f->count++;
	}
}

void fusion_slam(struct fusion_state *f, uint32_t vts_us, const float q_slam[4])
{
	/* Integrated orientation at the SLAM timestamp, interpolated between
	 * the samples around it (signed differences handle VTS wraparound). */
	const struct fusion_sample *at = NULL, *after = NULL;
	for (int n = 1; n <= f->count; n++) {
		const struct fusion_sample *s = &f->history[(f->head - n + FUSION_HISTORY) % FUSION_HISTORY];
		if ((int32_t)(s->time_us - vts_us) <= 0) {
			at = s;
			break;
		}
		after = s;
	}
	if (at == NULL) {
		/* No IMU history yet: start from the pose itself. A pose older
		 * than the history (a USB stall) is skipped. */
		if (!f->initialized) {
			memcpy(f->q, q_slam, sizeof(f->q));
			quat_normalize(f->q);
			f->initialized = 1;
		}
		return;
	}

	float then[4];
	memcpy(then, at->q, sizeof(then));
	if (after != NULL && after->time_us != at->time_us) {
		float t = (float)(int32_t)(vts_us - at->time_us) / (float)(int32_t)(after->time_us - at->time_us);
		float sign = at->q[0] * after->q[0] + at->q[1] * after->q[1]
			+ at->q[2] * after->q[2] + at->q[3] * after->q[3] < 0.0f ? -1.0f : 1.0f;
		for (int i = 0; i < 4; i++) {
			then[i] = at->q[i] * (1.0f - t) + after->q[i] * sign * t;
		}
		quat_normalize(then);
	}

	/* World-frame difference between SLAM and the integration at that time */
	float conj[4] = {then[0], -then[1], -then[2], -then[3]};
	float diff[4], error[3];
	quat_mul(q_slam, conj, diff);
	quat_log(diff, error);
	float angle = sqrtf(error[0] * error[0] + error[1] * error[1] + error[2] * error[2]);
	f->last_error = angle;

	float correction[4];
	if (!f->initialized || angle > FUSION_SNAP_RAD) {
		quat_exp(error, correction);
		f->initialized = 1;
	} else {
		float part[3];
		for (int i = 0; i < 3; i++) {
			part[i] = error[i] * FUSION_KP;
		}
		quat_exp(part, correction);
		/* The gyro measures in the body frame */
		float body[3];
		rotate_inverse(then, error, body);
		for (int i = 0; i < 3; i++) {
			float b = f->bias[i] - FUSION_KI * body[i];
			f->bias[i] = fmaxf(-FUSION_BIAS_LIMIT, fminf(FUSION_BIAS_LIMIT, b));
		}
	}

	/* Rotate the present and the whole history, so the next SLAM pose is
	 * compared with orientations that already include this correction. */
	quat_mul(correction, f->q, f->q);
	quat_normalize(f->q);
	for (int n = 1; n <= f->count; n++) {
		struct fusion_sample *s = &f->history[(f->head - n + FUSION_HISTORY) % FUSION_HISTORY];
		quat_mul(correction, s->q, s->q);
	}
}

void fusion_predict(const struct fusion_state *f, float ahead, float out[4])
{
	float step[3], dq[4];
	for (int i = 0; i < 3; i++) {
		step[i] = f->rate[i] * ahead;
	}
	quat_exp(step, dq);
	quat_mul(f->q, dq, out);
	quat_normalize(out);
}
