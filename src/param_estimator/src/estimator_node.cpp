//
// Created by lay on 24-1-23.
//
#include <chrono>
#include <memory>
#include <Eigen/Eigen>
#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/actuator_outputs.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "px4_msgs/msg/vehicle_angular_velocity.hpp"
#include "px4_msgs/msg/vehicle_attitude.hpp"
#include "px4_msgs/msg/vehicle_thrust_setpoint.hpp"
#include "px4_msgs/msg/vehicle_torque_setpoint.hpp"
#include "px4_msgs/msg/actuator_motors.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "online_param_estimator/msg/params.hpp"
#include "EKF_estimator/EKF_estimator.h"
#include "vector"
//using std::placeholders::_1;
using namespace Eigen;

class Integral_filter {
    Vector3d last_output;
    Vector3d input;
    double alpha;
public:
    explicit Integral_filter(double alpha) : last_output(Vector3d::Zero()), input(Vector3d::Zero()), alpha(alpha) {}

    Vector3d update(const Vector3d &_input) {
        input = _input;
        Vector3d output = alpha * input + (1 - alpha) * last_output;
        last_output = output;
        return output;
    }
};

class Moving_avg_filter {
private:
    std::vector<Vector3d> window;  // 存储滑动窗口中的元素
    int size;                 // 窗口大小
    Vector3d avg_vector;                  // 窗口元素总和

public:
    Moving_avg_filter(int window_size) : size(window_size) {}

    Vector3d update(const Vector3d &_input) {
        if (window.size() >= size) {
            window.erase(window.begin());
        }
        window.push_back(_input);
        avg_vector = Vector3d::Zero();
        for (const auto &i: window) {
            avg_vector += i;
        }
        avg_vector /= window.size();
        return avg_vector;
    }
};

Vector3d acc, ang_vel, ang_acc, torque, thrust;
Vector4d actuator, actuator_sim, actuator_cmd;
Quaterniond att;
double sample_T = 0.005;

EKF_estimator ekfEstimator(sample_T);
Integral_filter torque_filter_1(0.01);
Moving_avg_filter torque_filter_2(20);
rclcpp::Publisher<online_param_estimator::msg::Params>::SharedPtr param_pub;
rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr inputs_pub;

void calculate_inputs(Inputs &_inputs);

inline void actuator_cb(const px4_msgs::msg::ActuatorOutputs::SharedPtr msg) {
    actuator << msg->output[0], msg->output[1], msg->output[2], msg->output[3];
    auto bias = Vector4d(1000, 1000, 1000, 1000);
    actuator -= bias;
}

inline void actuator_sim_cb(const px4_msgs::msg::ActuatorOutputs::SharedPtr msg) {
    actuator_sim << msg->output[0], msg->output[1], msg->output[2], msg->output[3];
}

inline void actuator_cmd_cb(const px4_msgs::msg::ActuatorMotors::SharedPtr msg) {
    actuator_cmd << msg->control[0], msg->control[1], msg->control[2], msg->control[3];
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
    auto pos_sub = node->create_subscription<px4_msgs::msg::VehicleLocalPosition>("/fmu/out/vehicle_local_position",
                                                                                  qos, &pos_cb);
    auto ang_vel_sub = node->create_subscription<px4_msgs::msg::VehicleAngularVelocity>(
            "/fmu/out/vehicle_angular_velocity", qos, &ang_vel_cb);
    auto att_sub = node->create_subscription<px4_msgs::msg::VehicleAttitude>("/fmu/out/vehicle_attitude", qos, &att_cb);
    auto thrust_sub = node->create_subscription<px4_msgs::msg::VehicleThrustSetpoint>(
            "/fmu/out/vehicle_thrust_setpoint", qos, &thrust_cb);
    auto torque_sub = node->create_subscription<px4_msgs::msg::VehicleTorqueSetpoint>(
            "/fmu/out/vehicle_torque_setpoint", qos, &torque_cb);
    auto actuator_sub = node->create_subscription<px4_msgs::msg::ActuatorOutputs>("/fmu/out/actuator_outputs", qos,
                                                                                  &actuator_cb);
    auto actuator_sim_sub = node->create_subscription<px4_msgs::msg::ActuatorOutputs>("/fmu/out/actuator_outputs_sim",
                                                                                      qos,
                                                                                      &actuator_sim_cb);
    auto actuator_cmd_sub = node->create_subscription<px4_msgs::msg::ActuatorMotors>("/fmu/out/actuator_motors", qos,
                                                                                     &actuator_cmd_cb);
    param_pub = node->create_publisher<online_param_estimator::msg::Params>("param_estimates", 10);
    inputs_pub = node->create_publisher<geometry_msgs::msg::Vector3>("inputs", 10);
    auto timer = node->create_wall_timer(
            std::chrono::milliseconds(int(sample_T * 1000)), [node]() {
                {
                    States measurements{};
                    measurements.acc = acc;
                    measurements.att = att;
                    measurements.ang_vel = ang_vel;
                    Inputs inputs{};
                    calculate_inputs(inputs);
                    {
                        geometry_msgs::msg::Vector3 msg;
                        msg.x = inputs.torque(0);
                        msg.y = inputs.torque(1);
                        msg.z = inputs.torque(2);
                        inputs_pub->publish(msg);
                    }
                    ekfEstimator.estimate(measurements, inputs);
                }
                {
                    Params param_estimates;
                    States state_estimates;
                    ekfEstimator.get_estimates(param_estimates, state_estimates);
                    online_param_estimator::msg::Params msg;
                    msg.header.stamp = node->get_clock()->now();
                    msg.m = param_estimates.mass;
                    msg.j.x = param_estimates.inertia(0);
                    msg.j.y = param_estimates.inertia(1);
                    msg.j.z = param_estimates.inertia(2);
                    msg.ang_vel.x = state_estimates.ang_vel(0);
                    msg.ang_vel.y = state_estimates.ang_vel(1);
                    msg.ang_vel.z = state_estimates.ang_vel(2);
                    param_pub->publish(msg);
//                    RCLCPP_INFO(rclcpp::get_logger("debug"), "---------%f", param_estimates.inertia(0));
                }
            });
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

void calculate_inputs(Inputs &_inputs) {
    Vector3d axes = {0, 0, -1};
    std::array<Vector3d, 4> rotor_pos = {Vector3d(0.15, 0.25, 0), Vector3d(-0.15, -0.19, 0), Vector3d(0.15, -0.25, 0),
                                         Vector3d(-0.15, 0.19, 0)};
    double ct = 6.5;
    double km[4] = {0.05, 0.05, -0.05, -0.05};
    actuator_sim = actuator_sim.array() * actuator_sim.array();//use \omega_i^2
    for (int i = 0; i < 4; i++) {
        _inputs.torque += (ct * rotor_pos[i].cross(axes) - ct * km[i] * axes) * actuator_sim(i);
        _inputs.thrust += actuator_sim(i);
    }
    _inputs.torque = torque_filter_2.update(_inputs.torque);
//    _inputs.torque=torque;
}



