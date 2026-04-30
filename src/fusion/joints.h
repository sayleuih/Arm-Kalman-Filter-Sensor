#pragma once

#define DOF 6

struct JointState {
    float q[DOF];     // joint angles
    float qdot[DOF];  // velocities
};