//
// Created by lay on 24-1-23.
//
#include <chrono>
#include <memory>
#include <Eigen/Eigen>
#include "rclcpp/rclcpp.hpp"
#include "param_estimator/type_def.h"
#include "px4_msgs/msg/actuator_outputs.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "px4_msgs/msg/vehicle_angular_velocity.hpp"
#include "px4_msgs/msg/vehicle_attitude.hpp"
#include "px4_msgs/msg/vehicle_thrust_setpoint.hpp"
#include "px4_msgs/msg/vehicle_torque_setpoint.hpp"
#include "online_param_estimator/msg/params.hpp"
#include "EKF_estimator/EKF_estimator.h"
//using std::placeholders::_1;
using namespace Eigen;

Vector3d acc, ang_vel, ang_acc, torque, thrust;
Vector4d actuator;
Quaterniond att;
double m;
Vector3d j;
EKF_estimator ekfEstimator;

rclcpp::Publisher<online_param_estimator::msg::Params>::SharedPtr param_pub;

inline void actuator_cb(const px4_msgs::msg::ActuatorOutputs::SharedPtr msg) {
    actuator << msg->output[0], msg->output[1], msg->output[2], msg->output[3];
    auto bias = Vector4d(1000, 1000, 1000, 1000);
    actuator -= bias;
}

inline void pos_cb(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
    acc << msg->ax, msg->ay, msg->az;//NED Frame
}

inline void ang_vel_cb(const px4_msgs::msg::VehicleAngularVelocity::SharedPtr msg) {
    ang_vel << msg->xyz[0], msg->xyz[1], msg->xyz[2];//body frame
    ang_acc << msg->xyz_derivative[0], msg->xyz_derivative[1], msg->xyz_derivative[2];
}

inline void att_cb(const px4_msgs::msg::VehicleAttitude::SharedPtr msg) {
    att = Quaterniond(msg->q[0], msg->q[1], msg->q[2], msg->q[3]);//from body to earth??
}

inline void thrust_cb(const px4_msgs::msg::VehicleThrustSetpoint::SharedPtr msg) {
    thrust << msg->xyz[0], msg->xyz[1], msg->xyz[2];//body frame FRD
}

inline void torque_cb(const px4_msgs::msg::VehicleTorqueSetpoint::SharedPtr msg) {
    torque << msg->xyz[0], msg->xyz[1], msg->xyz[2];//body frame FRD
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
    auto node = std::make_shared<rclcpp::Node>("param_estimator");
    auto actuator_sub = node->create_subscription<px4_msgs::msg::ActuatorOutputs>("/fmu/out/actuator_outputs", qos,
                                                                                  &actuator_cb);
    auto pos_sub = node->create_subscription<px4_msgs::msg::VehicleLocalPosition>("/fmu/out/vehicle_local_position",
                                                                                  qos, &pos_cb);
    auto ang_vel_sub = node->create_subscription<px4_msgs::msg::VehicleAngularVelocity>(
            "/fmu/out/vehicle_angular_velocity", qos, &ang_vel_cb);
    auto att_sub = node->create_subscription<px4_msgs::msg::VehicleAttitude>("/fmu/out/vehicle_attitude", qos, &att_cb);
    auto thrust_sub = node->create_subscription<px4_msgs::msg::VehicleThrustSetpoint>(
            "/fmu/out/vehicle_thrust_setpoint", qos, &thrust_cb);
    auto torque_sub = node->create_subscription<px4_msgs::msg::VehicleTorqueSetpoint>(
            "/fmu/out/vehicle_torque_setpoint", qos, &torque_cb);
    param_pub = node->create_publisher<online_param_estimator::msg::Params>("param_estimates", 10);
    auto timer = node->create_wall_timer(
            std::chrono::milliseconds(10), [node]() {
                States states{acc, ang_vel, ang_acc, att};
                Inputs inputs{torque, thrust, actuator};
                ekfEstimator.estimate(states, inputs);
                ekfEstimator.get_estimates(m, j);
                online_param_estimator::msg::Params msg;
                msg.header.stamp = node->get_clock()->now();
                msg.m = m;
                msg.jxx = j(0);
                msg.jyy = j(1);
                msg.jzz = j(2);
                param_pub->publish(msg);
                RCLCPP_INFO(rclcpp::get_logger("param_estimator"), "m:%3.4f,j:%3.4f,%3.4f,%3.4f", m, j(0), j(1), j(2));
            });
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}