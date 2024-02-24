//
// Created by lay on 24-1-25.
//

#ifndef BUILD_TYPE_DEF_H
#define BUILD_TYPE_DEF_H
#include "Eigen/Eigen"

struct States {
    Eigen::Vector3d acc, ang_vel, ang_acc;
    Eigen::Quaterniond att;
};
struct Inputs {
    Eigen::Vector3d torque, thrust;
    Eigen::Vector4d actuator;
};

#endif //BUILD_TYPE_DEF_H
