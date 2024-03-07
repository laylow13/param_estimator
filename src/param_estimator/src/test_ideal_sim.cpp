//
// Created by lay on 24-3-7.
//
//
// Created by lay on 24-3-1.
//
#include "EKF_estimator/EKF_estimator.h"
#include <iostream>

Vector3d random_M() {
    Vector3d ret;
    ret << rand() % 2 - 1, rand() % 2 - 1, rand() % 2 - 1;
    return ret;
}

Vector3d sin_M(int i) {
    Vector3d ret;
    ret << sin(i * 0.01), 1, cos(i * 0.01);
    return ret;
}

int main() {
    EKF_estimator my_estimator(0.01);
    {
        Initial_val initialVal{};
        initialVal.x2 = VectorXd::Ones(6, 1) * 1e-3;
        initialVal.var_x2 = Matrix<double, 6, 6>::Constant(1e-4) + Matrix<double, 6, 6>::Identity();
        initialVal.R_x2 = Matrix<double, 6, 6>::Identity() * 1e-4;
        initialVal.Q_x2 = Matrix3d::Identity() * 1e-4;
        my_estimator.reinit(initialVal);
    }
    States measurement{};
    Inputs input{};
    double mass_est;
    Vector3d inertia{0.9, 0.8, 0.5};
    Matrix3d inertia_mat;
    inertia_mat << inertia(0), 0, 0,
            0, inertia(1), 0,
            0, 0, inertia(2);
    Vector3d ang_vel{0.1, 0.1, 0.1};
    Vector3d torque{0.1, 0.1, 0.1};
    Vector3d inertia_est;
    Vector3d ang_vel_est;
    for (int i = 0; i < 300; ++i) {
        torque = sin_M(i);
        auto inertia_inv = inertia_mat.inverse();
        Vector3d ang_vel_dot = inertia_inv * torque - inertia_inv * (ang_vel.cross(inertia_mat * ang_vel));
        ang_vel += ang_vel_dot * 0.01;
        measurement.ang_vel = ang_vel;
        input.torque = torque;
        my_estimator.estimate(measurement, input);
        Params param_estimates;
        States state_estimates;
        my_estimator.get_estimates(param_estimates, state_estimates);
        std::cout << "estimate_j:" << param_estimates.inertia << "\n";
        std::cout << "estimate_ang_vel:" << state_estimates.ang_vel << "\n";
        std::cout << "estimate_ang_vel_err:" << ang_vel - state_estimates.ang_vel << "\n";
        std::cout << "*********************" << "\n";
    }
    return 0;
}
