#pragma once
#include "quaternion.h"
#include "joints.h"

struct DHParam {
    float a;
    float d;
    float alpha;
};

class DHModel {
public:
    DHModel();
    Quat forwardOrientation(int link, const JointState& js);

private:
    DHParam dh[DOF];
};