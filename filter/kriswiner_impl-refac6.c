#include<stdbool.h>

#include "madgwick_filter.h"

void set_beta(float _beta) {
    beta = _beta;
}

void set_deltat(float _deltat) {
    deltat = _deltat;
}

inline bool normalize_quat(quaternion_t *quat) {
    float norm = sqrtf(quat->a * quat->a + quat->b * quat->b + quat->c * quat->c + quat->d * quat->d);
    if (norm == 0.0f) {
        return false;
    }
    norm = 1.0f / norm;
    quat->a *= norm;
    quat->b *= norm;
    quat->c *= norm;
    quat->d *= norm;
    return true;
}

inline void scalar_quat(float f, quaternion_t *quat) {
    quat->a *= f;
    quat->b *= f;
    quat->c *= f;
    quat->d *= f;
}

inline void add_quat(quaternion_t *q1, quaternion_t q2) {
    q1->a += q2.a;
    q1->b += q2.b;
    q1->c += q2.c;
    q1->d += q2.d;
}

inline bool normalize_vec3(vec3_t *vector) {
    float norm = sqrtf(vector->x * vector->x + vector->y * vector->y + vector->z * vector->z);
    if (norm == 0.0f) {
        return false;
    }
    norm = 1.0f / norm;
    vector->x *= norm;
    vector->y *= norm;
    vector->z *= norm;
    return true;
}

quaternion_t differentiate_quat(vec3_t gyro, quaternion_t *quat) {
    quaternion_t dot;
    dot.a = 0.5f * (-quat->b * gyro.x - quat->c * gyro.y - quat->d * gyro.z) - beta * quat->a;
    dot.b = 0.5f * (quat->a  * gyro.x + quat->c * gyro.z - quat->d * gyro.y) - beta * quat->b;
    dot.c = 0.5f * (quat->a  * gyro.y - quat->b * gyro.z + quat->d * gyro.x) - beta * quat->c;
    dot.d = 0.5f * (quat->a  * gyro.z + quat->b * gyro.y - quat->c * gyro.x) - beta * quat->d;
    return dot;
}

quaternion_t madgwick_filter(vec3_t acc, vec3_t gyro, vec3_t mag) {
    set_deltat(0.01);

    quat.a = 1.0f;
    quat.b = 0.0f;
    quat.c = 0.0f;
    quat.d = 0.0f;

    float norm;
    float hx, hy, _2bx, _2bz;
    quaternion_t s;

    // Auxiliary variables to avoid repeated arithmetic
    float _2q1mx;
    float _2q1my;
    float _2q1mz;
    float _2q2mx;
    float _4bx;
    float _4bz;
    float _2q1   = 2.0f * quat.a;
    float _2q2   = 2.0f * quat.b;
    float _2q3   = 2.0f * quat.c;
    float _2q4   = 2.0f * quat.d;
    float _2q1q3 = 2.0f * quat.a * quat.c;
    float _2q3q4 = 2.0f * quat.c * quat.d;
    float q1q1   = quat.a * quat.a;
    float q1q2   = quat.a * quat.b;
    float q1q3   = quat.a * quat.c;
    float q1q4   = quat.a * quat.d;
    float q2q2   = quat.b * quat.b;
    float q2q3   = quat.b * quat.c;
    float q2q4   = quat.b * quat.d;
    float q3q3   = quat.c * quat.c;
    float q3q4   = quat.c * quat.d;
    float q4q4   = quat.d * quat.d;

    // Normalise accelerometer measurement
    if (!normalize_vec3(&acc)) {
        return quat;
    }

    // Normalise magnetometer measurement
    if (!normalize_vec3(&mag)) {
        return quat;
    }

    // Reference direction of Earth's magnetic field
    _2q1mx = 2.0f * quat.a * mag.x;
    _2q1my = 2.0f * quat.a * mag.y;
    _2q1mz = 2.0f * quat.a * mag.z;
    _2q2mx = 2.0f * quat.b * mag.x;
    hx   = mag.x * q1q1 - _2q1my * quat.d + _2q1mz * quat.c + mag.x * q2q2 + _2q2 * mag.y * quat.c + _2q2 * mag.z * quat.d - mag.x * q3q3 - mag.x * q4q4;
    hy   = _2q1mx * quat.d + mag.y * q1q1 - _2q1mz * quat.b + _2q2mx * quat.c - mag.y * q2q2 + mag.y * q3q3 + _2q3 * mag.z * quat.d - mag.y * q4q4;
    _2bx = sqrtf(hx * hx + hy * hy);
    _2bz = -_2q1mx * quat.c + _2q1my * quat.b + mag.z * q1q1 + _2q2mx * quat.d - mag.z * q2q2 + _2q3 * mag.y * quat.d - mag.z * q3q3 + mag.z * q4q4;
    _4bx = 2.0f * _2bx;
    _4bz = 2.0f * _2bz;

    // Gradient decent algorithm corrective step
    s.a = -_2q3 * (2.0f * q2q4 - _2q1q3 - acc.x) + _2q2 * (2.0f * q1q2 + _2q3q4 - acc.y) -
         _2bz * quat.c * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mag.x) +
         (-_2bx * quat.d + _2bz * quat.b) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - mag.y) +
         _2bx * quat.c * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mag.z);
    s.b = _2q4 * (2.0f * q2q4 - _2q1q3 - acc.x) + _2q1 * (2.0f * q1q2 + _2q3q4 - acc.y) -
         4.0f * quat.b * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - acc.z) +
         _2bz * quat.d * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mag.x) +
         (_2bx * quat.c + _2bz * quat.a) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - mag.y) +
         (_2bx * quat.d - _4bz * quat.b) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mag.z);
    s.c = -_2q1 * (2.0f * q2q4 - _2q1q3 - acc.x) + _2q4 * (2.0f * q1q2 + _2q3q4 - acc.y) -
         4.0f * quat.c * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - acc.z) +
         (-_4bx * quat.c - _2bz * quat.a) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mag.x) +
         (_2bx * quat.b + _2bz * quat.d) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - mag.y) +
         (_2bx * quat.a - _4bz * quat.c) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mag.z);
    s.d = _2q2 * (2.0f * q2q4 - _2q1q3 - acc.x) + _2q3 * (2.0f * q1q2 + _2q3q4 - acc.y) +
         (-_4bx * quat.d + _2bz * quat.b) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mag.x) +
         (-_2bx * quat.a + _2bz * quat.c) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - mag.y) +
         _2bx * quat.b * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mag.z);
    // normalise step magnitude
    if (!normalize_quat(&s)) {
        return quat;
    }

    // Compute rate of change of quaternion
    quaternion_t qdot = differentiate_quat(gyro, &s);

    // Integrate to yield quaternion
    scalar_quat(deltat, &qdot);

    add_quat(&quat, qdot);

    normalize_quat(&quat);
    
    return quat;
}
