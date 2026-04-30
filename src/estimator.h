#pragma once
#include "imu_data.h"
#include "joints.h"
#include "dh_model.h"

class Estimator {
public:
    Estimator();

    void update(float dt,
                const IMUData& base,
                const IMUData& arm,
                const IMUData& wrist);

    JointState get();

private:
    JointState joints;
    DHModel model;

    float gain = 0.05f;

    void integrate(float dt);
    void correctIMU(const IMUData& imu, int link);
    void relativeConstraint(const IMUData& base, const IMUData& arm);
};