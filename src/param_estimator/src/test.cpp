//
// Created by lay on 24-3-1.
//
#include "EKF_estimator/EKF_estimator.h"
#include "InertiaEstimatorEKF.h"
#include "rt_nonfinite.h"
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>
#include <iostream>

int main() {
    EKF_estimator my_estimator(0.01);
    InertiaEstimatorEKF InertiaEstimator;
    States measurement{};
    Inputs input{};
    double my_mass;
    Vector3d my_inertia;
    Vector3d my_ang_vel;
    double *M_in;
    double *ang_vel_in;
    double out[42];
    Matrix<double, 6, 6> tmp;
    tmp << 0.1, 0, 0, 0, 0, 0,
            0, 0.1, 0, 0, 0, 0,
            0, 0, 0.1, 0, 0, 0,
            0, 0, 0, 0.0001, 0, 0,
            0, 0, 0, 0, 0.0001, 0,
            0, 0, 0, 0, 0, 0.0001;
    std::cout << tmp << "\n";
    for (int i = 0; i < 100; ++i) {
        measurement.ang_vel << 1.0f, 1.0f, 1.0f;
        input.torque << 1.0f, 1.0f, 1.0f;
        measurement.ang_vel *= i;
        input.torque *= i;
        my_estimator.estimate(measurement, input);
        my_estimator.get_estimates(my_mass, my_inertia, my_ang_vel);
        std::cout << my_inertia << "\n";
        M_in = input.torque.data();
        ang_vel_in = measurement.ang_vel.data();
        InertiaEstimator.estimateInertia(M_in, 0.01, ang_vel_in, out);
        Vector3d inertia{out[3], out[4], out[5]};
        std::cout << inertia << "\n";
        std::cout << "*********************" << "\n";
    }

    return 0;
};