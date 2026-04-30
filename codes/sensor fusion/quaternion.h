#pragma once
#include <math.h>

struct Quat {
    float w, x, y, z;
};

inline Quat q_identity() { return {1,0,0,0}; }

inline Quat normalize(const Quat& q) {
    float n = sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
    return {q.w/n, q.x/n, q.y/n, q.z/n};
}

inline Quat qmul(const Quat& a, const Quat& b) {
    return {
        a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z,
        a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
        a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
        a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w
    };
}

inline Quat qinv(const Quat& q) {
    return {q.w, -q.x, -q.y, -q.z};
}

inline Quat axisAngle(float x, float y, float z, float angle) {
    float s = sin(angle/2.0f);
    return {cos(angle/2.0f), x*s, y*s, z*s};
}

inline void qtoaxis(const Quat& q, float &x, float &y, float &z) {
    float s = sqrt(1 - q.w*q.w);
    if (s < 1e-6) { x=y=z=0; return; }
    x = q.x / s;
    y = q.y / s;
    z = q.z / s;
}