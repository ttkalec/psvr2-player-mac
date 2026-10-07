/* Synthetic moving headset for the render-thread integration check.
 * This executable never opens USB or changes real headset state. */
#include "cpsvr2.h"
#include <math.h>
#include <stdatomic.h>
#include <time.h>

static atomic_int predictions;
int test_prediction_count(void) { return atomic_load(&predictions); }
int psvr2_start(void) { return 0; }
void psvr2_stop(void) {}
int psvr2_connected(void) { return 1; }
int psvr2_get_predicted_quat(float ahead, float q[4]) {
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    float half = (float)fmod((double)t.tv_sec + t.tv_nsec * 1e-9 + ahead, 4.0 * M_PI) * 0.5f;
    q[0] = cosf(half); q[1] = 0; q[2] = sinf(half); q[3] = 0;
    atomic_fetch_add(&predictions, 1);
    return 1;
}
int psvr2_get_pose(float q[4], float p[3]) {
    q[0] = 1; q[1] = q[2] = q[3] = 0;
    p[0] = p[1] = p[2] = 0;
    return 1;
}
int psvr2_get_status(int *prox, int *ipd) { *prox = 1; *ipd = 63; return 0; }
int psvr2_get_motion(float g[3], double *age) {
    g[0] = g[2] = 0; g[1] = 1; *age = 0.001; return 1;
}
int psvr2_get_fusion_status(float *bias, float *correction) { *bias = 0; *correction = 0; return 1; }
int psvr2_get_button(void) { return 0; }
int psvr2_set_brightness(float value) { (void)value; return 0; }
int psvr2_get_distortion_calibration(float out[8]) { (void)out; return -1; }
int psvr2_camera_start(void) { return -1; }
void psvr2_camera_stop(void) {}
int psvr2_camera_get_frame(unsigned char *l, unsigned char *r) { (void)l; (void)r; return 0; }
