//
// Created by lay on 24-1-23.
//

#include "EKF_estimator.h"

EKF_estimator::EKF_estimator() {
    m = 0.001f;
    var_m = 0.001f;
    R_m = 0.001f;
    Q_m = 1.0f;

    j << 0.01f, 0.01f, 0.01f;
    var_j << 0.01f, 0.001f, 0.001f,
            0.001f, 0.01f, 0.001f,
            0.001f, 0.001f, 0.01f;
    R_j << 0.01f, 0.001f, 0.001f,
            0.001f, 0.01f, 0.001f,
            0.001f, 0.001f, 0.01f;
    Q_j << 0.01f, 0.001f, 0.001f,
            0.001f, 0.01f, 0.001f,
            0.001f, 0.001f, 0.01f;
}

void EKF_estimator::estimate(const States &states, const Inputs &inputs) {
    pre_process(states, inputs);
    //1.predict
    var_m = var_m + R_m;
    var_j = var_j + R_j;
    //2.measurement
    calculate_H();
    calculate_K();

    m = m + K_m * (acc(2) - dynamic_m());
    var_m = (1.0f - K_m * H_m) * var_m;

    auto I = MatrixXd::Identity(3, 3);
    j = j + K_j * (ang_acc - dynamic_j());
    var_j = (I - K_j * H_j) * var_j;
}

void EKF_estimator::get_estimates(double &_m, Vector3d &_j) const {
    _m = m;
    _j = j;
}

void EKF_estimator::pre_process(const States &states, const Inputs &inputs) {
    acc = states.acc;
    ang_vel = states.ang_vel;
    ang_acc = states.ang_acc;
    att = states.att;
    torque = inputs.torque;
    thrust = inputs.thrust;
    actuator = inputs.actuator;

    actuator = actuator.array() * actuator.array();//use \omega_i^2
    auto &omega_1 = actuator(0);
    auto &omega_2 = actuator(1);
    auto &omega_3 = actuator(2);
    auto &omega_4 = actuator(3);
//    M << 1e-6*(omega_2 + omega_3 - omega_1 - omega_4), 1e-6*(omega_1 + omega_3 - omega_2 - omega_4), 1e-6*(omega_1 + omega_2 -
//                                                                                            omega_3 - omega_4);
    T = 1e-6*(omega_1 + omega_2 + omega_3 + omega_4);
//    T = thrust(2) > 0 ? 0 : thrust(2);
//    T *= -1.0f;
    M = torque;
}

void EKF_estimator::calculate_H() {
    Vector3d e_z{0, 0, 1};//[0 0 1]
    auto body_z = att.toRotationMatrix() * e_z;
    H_m = T * body_z(2) / (m * m);

    auto &jx = j(0);
    auto &jy = j(1);
    auto &jz = j(2);
    auto &avx = ang_vel(0);
    auto &avy = ang_vel(1);
    auto &avz = ang_vel(2);

    H_j(0, 0) = (jz - jy) / (jx * jx) * avy * avz - M(0) / (jx * jx);
    H_j(0, 1) = avy * avz / jx;
    H_j(0, 2) = -avy * avz / jx;

    H_j(1, 0) = -avx * avz / jy;
    H_j(1, 1) = (jx - jz) / (jy * jy) * avx * avz - M(1) / (jy * jy);
    H_j(1, 2) = avx * avz / jy;

    H_j(2, 0) = avy * avx / jz;
    H_j(2, 1) = -avy * avx / jz;
    H_j(2, 2) = (jy - jx) / (jz * jz) * avx * avy - M(2) / (jz * jz);
}

void EKF_estimator::calculate_K() {
    K_m = var_m * H_m / (H_m * var_m * H_m + Q_m);
    auto var_innov = H_j * var_j * H_j.transpose() + Q_j;
    K_j = var_j * H_j.transpose() * var_innov.inverse();
}

double EKF_estimator::dynamic_m() {
    Vector3d e_z{0, 0, 1};//[0 0 1]
    auto body_z = att.toRotationMatrix() * e_z;
    double acc_hat = -T * body_z(2) / m + 9.8;
    return acc_hat;
}

Vector3d EKF_estimator::dynamic_j() {
    Matrix3d inertia;
    inertia << j(0), 0, 0,
            0, j(1), 0,
            0, 0, j(2);
    auto inertia_inv = inertia.inverse();
    Vector3d ang_acc_hat = inertia_inv * M - inertia_inv * (ang_vel.cross(inertia * ang_vel));
    return ang_acc_hat;
}
