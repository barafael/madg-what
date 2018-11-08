#ifndef MADGWICK_FILTER_H
#define MADGWICK_FILTER_H

#include "stdio.h"
#include <math.h>

typedef struct {
    float a;
    float b;
    float c;
    float d;
} quaternion_t;

typedef struct {
    float x;
    float y;
    float z;
} axis_t;

quaternion_t quat_from_array(float *q) {
    quaternion_t quat;
    quat.a = q[0];
    quat.b = q[1];
    quat.c = q[2];
    quat.d = q[3];
    return quat;
}

// gyroscope measurement error in rads/s (start at 40 deg/s)
// float GyroMeasError = M_PI * (40.0f / 180.0f);
float gyro_meas_error = 6.981317008f;
// gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
//float GyroMeasDrift = M_PI * (0.0f  / 180.0f); // yes, really
float gyro_meas_drift = 0.0f;
//float beta = sqrt(3.0f / 4.0f) * GyroMeasError;
float beta = 8.384266471f;
float deltat = 0.0f;
quaternion_t quat;

void set_beta(float beta);
void set_deltat(float deltat);
void set_quaternion(quaternion_t quat);

quaternion_t madgwick_filter(axis_t acc, axis_t gyro, axis_t mag);

#endif // MADGWICK_FILTER_H
