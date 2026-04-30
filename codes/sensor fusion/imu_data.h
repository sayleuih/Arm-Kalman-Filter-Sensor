#pragma once
#include <cstdint>

struct IMUData {
    float q[4];     // quaternion (w,x,y,z)
    float gyro[3];  // rad/s
    float accel[3]; // m/s^2
    uint32_t t;
};