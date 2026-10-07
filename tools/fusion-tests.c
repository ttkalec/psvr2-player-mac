/*
 * Head-orientation fusion against a known trajectory. A synthetic gyro has
 * the PS VR2's measured traits (3.3 deg/s bias, 2000 Hz in pairs sharing a
 * 1 ms timestamp) plus a 2% scale error and noise; SLAM poses arrive at
 * 63 Hz, 22 ms after their timestamp. The renderer samples a 25 ms
 * prediction at 120 Hz. Jitter is how much the error against the true pose
 * changes from one frame to the next: what the eye sees as shimmer/judder.
 * The baseline is the previous approach: restart integration at every SLAM
 * pose.
 */
#include "fusion.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define DEG ((float)M_PI / 180.0f)

static void qmul(const double a[4], const double b[4], double o[4])
{
	double w = a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3];
	double x = a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2];
	double y = a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1];
	double z = a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0];
	o[0] = w; o[1] = x; o[2] = y; o[3] = z;
}

static void qaxis(double angle, int axis, double o[4])
{
	memset(o, 0, 4 * sizeof(double));
	o[0] = cos(angle / 2);
	o[axis + 1] = sin(angle / 2);
}

static double amplitude, period;
/* Gyro scale error and SLAM timestamp offset of the current scenario */
static double gyro_scale, slam_offset_us;

/* Truth: yaw swings with an independent, slower pitch nod. */
static void truth(double t, double o[4])
{
	double yaw[4], pitch[4];
	qaxis(amplitude * sin(2 * M_PI * t / period), 1, yaw);
	qaxis(0.3 * amplitude * sin(2 * M_PI * t / (period * 1.7)), 0, pitch);
	qmul(yaw, pitch, o);
}

static void body_rate(double t, double w[3])
{
	double a[4], b[4], c[4], h = 1e-6;
	truth(t, a);
	truth(t + h, b);
	double conj[4] = {a[0], -a[1], -a[2], -a[3]};
	qmul(conj, b, c);
	double s = c[0] < 0 ? -2.0 / h : 2.0 / h;
	w[0] = c[1] * s; w[1] = c[2] * s; w[2] = c[3] * s;
}

/* Angle of a rotation from its vector part: acos(w) cannot resolve the
 * hundredths of a degree this test is about. */
static double rotation_angle(const double q[4])
{
	return 2 * atan2(sqrt(q[1] * q[1] + q[2] * q[2] + q[3] * q[3]), fabs(q[0]));
}

static double gauss(void)
{
	double u = (rand() + 1.0) / (RAND_MAX + 2.0), v = (rand() + 1.0) / (RAND_MAX + 2.0);
	return sqrt(-2 * log(u)) * cos(2 * M_PI * v);
}

/* Previous approach: last SLAM pose, then every newer gyro sample, then the
 * newest raw rate for the remaining time. */
struct baseline {
	float slam[4];
	uint32_t slam_vts;
	int have;
	uint32_t vts[256];
	float gyro[256][3];
	int head, count;
};

static void float_mul(const float a[4], const float b[4], float o[4])
{
	double da[4] = {a[0], a[1], a[2], a[3]}, db[4] = {b[0], b[1], b[2], b[3]}, r[4];
	qmul(da, db, r);
	for (int i = 0; i < 4; i++) o[i] = (float)r[i];
}

static void float_exp(const float v[3], float o[4])
{
	float a = sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
	float s = a < 1e-9f ? 0.5f : sinf(a / 2) / a;
	o[0] = cosf(a / 2); o[1] = v[0] * s; o[2] = v[1] * s; o[3] = v[2] * s;
}

static void baseline_predict(const struct baseline *b, float ahead, float out[4])
{
	float q[4] = {b->slam[0], b->slam[1], b->slam[2], b->slam[3]}, d[4];
	for (int n = b->count; n >= 1; n--) {
		int i = (b->head - n + 256) % 256;
		if ((int32_t)(b->vts[i] - b->slam_vts) <= 0) continue;
		float step[3] = {b->gyro[i][0] * 0.0005f, b->gyro[i][1] * 0.0005f, b->gyro[i][2] * 0.0005f};
		float_exp(step, d);
		float_mul(q, d, q);
	}
	int last = (b->head - 1 + 256) % 256;
	float step[3] = {b->gyro[last][0] * ahead, b->gyro[last][1] * ahead, b->gyro[last][2] * ahead};
	float_exp(step, d);
	float_mul(q, d, out);
}

struct result {
	double rms_error, jitter_rms, jitter_p99;
};

static int cmp(const void *a, const void *b)
{
	double x = *(const double *)a, y = *(const double *)b;
	return x < y ? -1 : x > y;
}

static void run(double amp_deg, struct result *fused, struct result *base)
{
	amplitude = amp_deg * M_PI / 180;
	period = 1.5;
	srand(7);
	struct fusion_state f;
	fusion_reset(&f);
	static struct baseline b;
	memset(&b, 0, sizeof(b));

	const double bias[3] = {-2.3 * DEG, -2.0 * DEG, 1.0 * DEG}; /* 3.3 deg/s, as measured */
	const double noise = 0.002;
	const double scale = gyro_scale, imu_dt = 0.0005, slam_dt = 1.0 / 63, latency = 0.022;
	const double frame_dt = 1.0 / 120, lookahead = 0.025, seconds = 20, settle = 6;
	double next_slam = 0.01, next_frame = 0;
	struct { double t; double q[4]; } pending[8];
	int npending = 0;

	int frames = 0, cap = (int)(seconds / frame_dt) + 2;
	double *err_f = calloc(cap, sizeof(double)), *err_b = calloc(cap, sizeof(double));
	double *jit_f = calloc(cap, sizeof(double)), *jit_b = calloc(cap, sizeof(double));
	double prev_f[4] = {1, 0, 0, 0}, prev_b[4] = {1, 0, 0, 0};
	int have_prev = 0, nj = 0;
	double sum_f = 0, sum_b = 0;

	for (long k = 0; k * imu_dt < seconds; k++) {
		double t = k * imu_dt;
		/* Pairs of samples share a 1 ms timestamp, like the headset */
		uint32_t vts = (uint32_t)((k / 2) * 1000);
		double w[3];
		body_rate(t, w);
		float g[3];
		for (int i = 0; i < 3; i++) g[i] = (float)(w[i] * scale + bias[i] + gauss() * noise);
		fusion_imu(&f, vts, g, (float)imu_dt);
		b.vts[b.head] = vts;
		memcpy(b.gyro[b.head], g, sizeof(g));
		b.head = (b.head + 1) % 256;
		if (b.count < 256) b.count++;

		if (t >= next_slam) {
			truth(t, pending[npending].q);
			/* SLAM noise, ~0.005 deg */
			double n[4] = {1, gauss() * 4e-5, gauss() * 4e-5, gauss() * 4e-5}, r[4];
			qmul(pending[npending].q, n, r);
			memcpy(pending[npending].q, r, sizeof(r));
			pending[npending++].t = t;
			next_slam += slam_dt;
		}
		while (npending > 0 && pending[0].t + latency <= t) {
			float q[4] = {(float)pending[0].q[0], (float)pending[0].q[1], (float)pending[0].q[2], (float)pending[0].q[3]};
			uint32_t svts = (uint32_t)(pending[0].t * 1e6 + slam_offset_us);
			fusion_slam(&f, svts, q);
			memcpy(b.slam, q, sizeof(q));
			b.slam_vts = svts;
			b.have = 1;
			memmove(pending, pending + 1, (npending - 1) * sizeof(pending[0]));
			npending--;
		}

		if (t >= next_frame && b.have) {
			next_frame += frame_dt;
			double ahead = lookahead + imu_dt;
			float qf[4], qb[4];
			fusion_predict(&f, (float)ahead, qf);
			baseline_predict(&b, (float)ahead, qb);
			double target[4];
			truth(t + ahead, target);
			/* Error rotation against the truth, per method */
			double ef[4], eb[4], conj[4] = {target[0], -target[1], -target[2], -target[3]};
			double dqf[4] = {qf[0], qf[1], qf[2], qf[3]}, dqb[4] = {qb[0], qb[1], qb[2], qb[3]};
			qmul(conj, dqf, ef);
			qmul(conj, dqb, eb);
			if (t > settle) {
				err_f[frames] = rotation_angle(ef);
				err_b[frames] = rotation_angle(eb);
				sum_f += err_f[frames] * err_f[frames];
				sum_b += err_b[frames] * err_b[frames];
				frames++;
				if (have_prev) {
					/* Change of the error between frames */
					double cf[4] = {prev_f[0], -prev_f[1], -prev_f[2], -prev_f[3]}, df[4];
					double cb[4] = {prev_b[0], -prev_b[1], -prev_b[2], -prev_b[3]}, db[4];
					qmul(cf, ef, df);
					qmul(cb, eb, db);
					jit_f[nj] = rotation_angle(df);
					jit_b[nj] = rotation_angle(db);
					nj++;
				}
				have_prev = 1;
			}
			memcpy(prev_f, ef, sizeof(ef));
			memcpy(prev_b, eb, sizeof(eb));
		}
	}
	double jsf = 0, jsb = 0;
	for (int i = 0; i < nj; i++) { jsf += jit_f[i] * jit_f[i]; jsb += jit_b[i] * jit_b[i]; }
	qsort(jit_f, nj, sizeof(double), cmp);
	qsort(jit_b, nj, sizeof(double), cmp);
	fused->rms_error = sqrt(sum_f / frames) / DEG;
	base->rms_error = sqrt(sum_b / frames) / DEG;
	fused->jitter_rms = sqrt(jsf / nj) / DEG;
	base->jitter_rms = sqrt(jsb / nj) / DEG;
	fused->jitter_p99 = jit_f[nj * 99 / 100] / DEG;
	base->jitter_p99 = jit_b[nj * 99 / 100] / DEG;
	free(err_f); free(err_b); free(jit_f); free(jit_b);
}

static int failures;

static void require(int ok, const char *message)
{
	if (!ok) {
		fprintf(stderr, "FAIL: %s\n", message);
		failures++;
	}
}

int main(void)
{
	struct result fused, base;
	gyro_scale = 1.0;
	slam_offset_us = 0;
	run(0, &fused, &base);
	printf("at rest:                    fused error %.4f deg, jitter p99 %.4f | restart-at-SLAM error %.4f, jitter p99 %.4f\n",
	       fused.rms_error, fused.jitter_p99, base.rms_error, base.jitter_p99);
	require(fused.jitter_p99 < base.jitter_p99 * 0.5, "at rest, fusion must remove most of the SLAM-rate jitter");
	require(fused.rms_error < 0.03, "at rest, fusion must converge onto the SLAM pose");

	/* The real gyro scale error and IMU/SLAM clock alignment are unknown:
	 * cover a 2% scale error and +-0.4 ms offsets. */
	const double scales[] = {1.0, 1.02};
	const double offsets[] = {-400, 0, 400};
	for (int i = 0; i < 2; i++) {
		for (int j = 0; j < 3; j++) {
			gyro_scale = scales[i];
			slam_offset_us = offsets[j];
			run(60, &fused, &base);
			printf("head turns, scale %.2f, %+4.0f us: fused error %.4f deg, jitter p99 %.4f | restart-at-SLAM error %.4f, jitter p99 %.4f\n",
			       gyro_scale, slam_offset_us, fused.rms_error, fused.jitter_p99, base.rms_error, base.jitter_p99);
			require(fused.jitter_p99 < base.jitter_p99 * 0.5, "during head turns, fusion must halve frame-to-frame jitter");
			require(fused.rms_error < base.rms_error * 1.25, "during head turns, fusion must stay about as accurate");
		}
	}

	if (failures == 0) {
		printf("PASS: fusion is smoother than restarting at each SLAM pose, at rest and in motion\n");
	}
	return failures != 0;
}
