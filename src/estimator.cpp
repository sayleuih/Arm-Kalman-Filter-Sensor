#include "estimator.h"
#include "quaternion.h"

Estimator::Estimator() {
    for(int i = 0;i < DOF; i++){
        joints.q[i] = 0;
        joints.qdot[i] = 0;
    }
}

void Estimator::integrate(float dt) {
    for(int i = 0; i < DOF; i++){
        joints.q[i] += joints.qdot[i] * dt;
    }
}

void Estimator::correctIMU(const IMUData& imu, int link) {

    Quat q_meas = {imu.q[0], imu.q[1], imu.q[2], imu.q[3]};
    q_meas = normalize(q_meas);

    Quat q_fk = model.forwardOrientation(link, joints);

    Quat err = qmul(q_meas, qinv(q_fk));

    float ex, ey, ez;
    qtoaxis(err, ex, ey, ez);

    for(int i=0;i<=link;i++){
        joints.q[i] += gain * ez;
    }
}

void Estimator::relativeConstraint(const IMUData& base, const IMUData& arm) {

    Quat qb = {base.q[0], base.q[1], base.q[2], base.q[3]};
    Quat qa = {arm.q[0], arm.q[1], arm.q[2], arm.q[3]};

    qb = normalize(qb);
    qa = normalize(qa);

    Quat rel_meas = qmul(qa, qinv(qb));

    Quat fk_b = model.forwardOrientation(0, joints);
    Quat fk_a = model.forwardOrientation(2, joints);

    Quat rel_fk = qmul(fk_a, qinv(fk_b));

    Quat err = qmul(rel_meas, qinv(rel_fk));

    float ex, ey, ez;
    qtoaxis(err, ex, ey, ez);

    joints.q[1] += gain * ez;
    joints.q[2] += gain * ez;
}

void Estimator::update(float dt,
                       const IMUData& base,
                       const IMUData& arm,
                       const IMUData& wrist) {

    integrate(dt);

    relativeConstraint(base, arm);

    correctIMU(base, 0);
    correctIMU(arm, 2);
    correctIMU(wrist, 5);
}

JointState Estimator::get() {
    return joints;
}