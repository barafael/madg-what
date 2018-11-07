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

// gyroscope measurement error in rads/s (start at 40 deg/s)
// float GyroMeasError = M_PI * (40.0f / 180.0f);
float gyro_meas_error = 6.981317008f;
// gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
//float GyroMeasDrift = M_PI * (0.0f  / 180.0f); // yes, really
float gyro_meas_drift = 0.0f;
//float beta = sqrt(3.0f / 4.0f) * GyroMeasError;
float beta = 8.384266471f;
float deltat = 0.0f;
float q[4];

void set_beta(float beta);
void set_deltat(float deltat);

quaternion_t madgwick_filter(axis_t acc, axis_t gyro, axis_t mag);

#endif // MADGWICK_FILTER_H
