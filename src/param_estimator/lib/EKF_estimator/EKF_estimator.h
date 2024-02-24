//
// Created by lay on 24-1-25.
//

#ifndef BUILD_EKF_ESTIMATOR_H
#define BUILD_EKF_ESTIMATOR_H

#include "param_estimator/type_def.h"
using namespace Eigen;

class EKF_estimator {
public:
    EKF_estimator();

    void estimate(const States &states,const Inputs &inputs);

    void get_estimates(double &_m, Vector3d &_j) const;

private:
    Vector3d acc, ang_vel, ang_acc;
    Quaterniond att;
    Vector3d torque, thrust, M;//torque~px4 torque;thrust~px4 thrust;M~moments to use
    double T;//thrust to use
    Vector4d actuator;// px4 actuator output
    double m;//mass
    Vector3d j;//inertia
    double var_m, R_m, H_m, K_m, Q_m;
    Matrix3d var_j, R_j, H_j, K_j, Q_j;

    void pre_process(const States &states,const Inputs &inputs);

    void calculate_H();

    void calculate_K();

    double dynamic_m();

    Vector3d dynamic_j();
};

#endif //BUILD_EKF_ESTIMATOR_H
