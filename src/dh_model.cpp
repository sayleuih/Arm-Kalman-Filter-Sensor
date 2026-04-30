#include "dh_model.h"

DHModel::DHModel() {
    // a, d, alpha
    dh[0] = {0, 131.22, M_PI/2};
    dh[1] = {-110.4, 0, 0};
    dh[2] = {-110.4, 0, 0};
    dh[3] = {0, 63.4, M_PI/2};
    dh[4] = {0, 75.05, -M_PI/2};
    dh[5] = {0, 45.6, 0};
}

Quat DHModel::forwardOrientation(int link, const JointState& js) {
    Quat q = q_identity();

    for (int i = 0; i <= link; i++) {
        Quat qz = axisAngle(0, 0, 1, js.q[i]);
        Quat qx = axisAngle(1, 0, 0, dh[i].alpha);

        q = qmul(q, qz);
        q = qmul(q, qx);
    }

    return normalize(q);
}