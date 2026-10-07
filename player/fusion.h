/* Head-orientation fusion: gyro integration corrected toward SLAM poses.
 * Pure functions on a state struct; cpsvr2.c calls them under its lock. */
#pragma once

#include <stdint.h>

/* ~128 ms of IMU samples at 2000 Hz: SLAM poses arrive ~20-35 ms old. */
#define FUSION_HISTORY 256

struct fusion_sample {
	uint32_t time_us; /* VTS clock, at the middle of the sample interval */
	float q[4];
};

struct fusion_state {
	int initialized;
	/* Orientation at the newest IMU sample, w,x,y,z in Monado-mapped axes. */
	float q[4];
	/* Estimated gyro bias and the newest bias-corrected rate, rad/s. */
	float bias[3];
	float rate[3];
	struct fusion_sample history[FUSION_HISTORY];
	int head;
	int count;
	uint32_t last_vts_us;
	int pair_index;
	/* Size of the last SLAM correction before smoothing, radians. */
	float last_error;
};

void fusion_reset(struct fusion_state *f);

/* One gyro sample (rad/s, mapped axes) covering dt seconds. */
void fusion_imu(struct fusion_state *f, uint32_t vts_us, const float gyro[3], float dt);

/* A SLAM orientation (w,x,y,z, mapped axes) measured at vts_us. */
void fusion_slam(struct fusion_state *f, uint32_t vts_us, const float q[4]);

/* Orientation extrapolated ahead seconds past the newest IMU sample. */
void fusion_predict(const struct fusion_state *f, float ahead, float out[4]);
