#include "madgwick_filter.h"

void set_beta(float _beta) {
    beta = _beta;
}

void set_deltat(float _deltat) {
    deltat = _deltat;
}

float magnitude3(vec3_t vector) {
    return sqrtf(vector.x * vector.x + vector.y * vector.y + vector.z * vector.z);
}

float magnitude4(float a, float b, float c, float d) {
    return sqrtf(a * a + b * b + c * c + d * d);
}

float magnitude_quat(quaternion_t quat) {
    return sqrtf(quat.a * quat.a + quat.b * quat.b + quat.c * quat.c + quat.d * quat.d);
}

quaternion_t differentiate_quat(vec3_t gyro, float a, float b, float c, float d) {
    quaternion_t dot;
    dot.a = 0.5f * (-b * gyro.x - c * gyro.y - d * gyro.z) - beta * a;
    dot.b = 0.5f * (a  * gyro.x + c * gyro.z - d * gyro.y) - beta * b;
    dot.c = 0.5f * (a  * gyro.y - b * gyro.z + d * gyro.x) - beta * c;
    dot.d = 0.5f * (a  * gyro.z + b * gyro.y - c * gyro.x) - beta * d;
    return dot;
}

quaternion_t madgwick_filter(vec3_t acc, vec3_t gyro, vec3_t mag) {
    set_deltat(0.01);

    quat.a = 1.0f;
    quat.b = 0.0f;
    quat.c = 0.0f;
    quat.d = 0.0f;

    float q1 = quat.a, q2 = quat.b, q3 = quat.c, q4 = quat.d;
    float norm;
    float hx, hy, _2bx, _2bz;
    float s1, s2, s3, s4;

    // Auxiliary variables to avoid repeated arithmetic
    float _2q1mx;
    float _2q1my;
    float _2q1mz;
    float _2q2mx;
    float _4bx;
    float _4bz;
    float _2q1   = 2.0f * q1;
    float _2q2   = 2.0f * q2;
    float _2q3   = 2.0f * q3;
    float _2q4   = 2.0f * q4;
    float _2q1q3 = 2.0f * q1 * q3;
    float _2q3q4 = 2.0f * q3 * q4;
    float q1q1   = q1 * q1;
    float q1q2   = q1 * q2;
    float q1q3   = q1 * q3;
    float q1q4   = q1 * q4;
    float q2q2   = q2 * q2;
    float q2q3   = q2 * q3;
    float q2q4   = q2 * q4;
    float q3q3   = q3 * q3;
    float q3q4   = q3 * q4;
    float q4q4   = q4 * q4;

    // Normalise accelerometer measurement
    norm = magnitude3(acc);
    if (norm == 0.0f) {
        return quat; // handle NaN
    }
    norm = 1.0f / norm;
    acc.x *= norm;
    acc.y *= norm;
    acc.z *= norm;

    // Normalise magnetometer measurement
    norm = magnitude3(mag);
    if (norm == 0.0f) {
        return quat; // handle NaN
    }
    norm = 1.0f / norm;
    mag.x *= norm;
    mag.y *= norm;
    mag.z *= norm;

    // Reference direction of Earth's magnetic field
    _2q1mx = 2.0f * q1 * mag.x;
    _2q1my = 2.0f * q1 * mag.y;
    _2q1mz = 2.0f * q1 * mag.z;
    _2q2mx = 2.0f * q2 * mag.x;
    hx   = mag.x * q1q1 - _2q1my * q4 + _2q1mz * q3 + mag.x * q2q2 + _2q2 * mag.y * q3 + _2q2 * mag.z * q4 - mag.x * q3q3 - mag.x * q4q4;
    hy   = _2q1mx * q4 + mag.y * q1q1 - _2q1mz * q2 + _2q2mx * q3 - mag.y * q2q2 + mag.y * q3q3 + _2q3 * mag.z * q4 - mag.y * q4q4;
    _2bx = sqrtf(hx * hx + hy * hy);
    _2bz = -_2q1mx * q3 + _2q1my * q2 + mag.z * q1q1 + _2q2mx * q4 - mag.z * q2q2 + _2q3 * mag.y * q4 - mag.z * q3q3 + mag.z * q4q4;
    _4bx = 2.0f * _2bx;
    _4bz = 2.0f * _2bz;

    // Gradient decent algorithm corrective step
    s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - acc.x) + _2q2 * (2.0f * q1q2 + _2q3q4 - acc.y) -
         _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mag.x) +
         (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - mag.y) +
         _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mag.z);
    s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - acc.x) + _2q1 * (2.0f * q1q2 + _2q3q4 - acc.y) -
         4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - acc.z) +
         _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mag.x) +
         (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - mag.y) +
         (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mag.z);
    s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - acc.x) + _2q4 * (2.0f * q1q2 + _2q3q4 - acc.y) -
         4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - acc.z) +
         (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mag.x) +
         (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - mag.y) +
         (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mag.z);
    s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - acc.x) + _2q3 * (2.0f * q1q2 + _2q3q4 - acc.y) +
         (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mag.x) +
         (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - mag.y) +
         _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mag.z);
    // normalise step magnitude
    norm = magnitude4(s1, s2, s3, s4);
    norm = 1.0f / norm;
    s1 *= norm;
    s2 *= norm;
    s3 *= norm;
    s4 *= norm;

    // Compute rate of change of quaternion
    quaternion_t qdot = differentiate_quat(gyro, s1, s2, s3, s4);

    // Integrate to yield quaternion
    q1 += qdot.a * deltat;
    q2 += qdot.b * deltat;
    q3 += qdot.c * deltat;
    q4 += qdot.d * deltat;

    norm = magnitude4(q1, q2, q3, q4);
    norm = 1.0f / norm;

    quat.a = q1 * norm;
    quat.b = q2 * norm;
    quat.c = q3 * norm;
    quat.d = q4 * norm;
    
    return quat;
}
