#pragma once
#include "lightweight_lidar_only_simulator/pose_2d.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace lightweight_lidar_only_simulator {

class ImuSimulator {
  private:
    Pose2D last_pose;
    double last_linear_velocity;
    double last_angular_velocity;

  public:
    ImuSimulator() {}

    ImuSimulator(Pose2D pose, double linear_velocity, double angular_velocity) 
    : last_pose(pose), last_linear_velocity(linear_velocity), last_angular_velocity(angular_velocity) {}

    sensor_msgs::msg::Imu get_reading_given_new_state(const Pose2D pose, double linear_velocity, double angular_velocity, double dt) {
        sensor_msgs::msg::Imu imu_msg;
        imu_msg.linear_acceleration.x = (linear_velocity - last_linear_velocity) / dt;
        imu_msg.linear_acceleration.y = (angular_velocity * linear_velocity);

        imu_msg.linear_acceleration_covariance[6] = -1; // not measuring z linear acceleration

        imu_msg.angular_velocity.z = angular_velocity;

        imu_msg.angular_velocity_covariance[0] = -1; // not measuring x angular velocity
        imu_msg.angular_velocity_covariance[3] = -1; // not measuring y angular velocity

        imu_msg.orientation_covariance[0] = -1; // not measuring x orientation
        imu_msg.orientation_covariance[3] = -1; // not measuring y orientation
        imu_msg.orientation_covariance[6] = -1; // not measuring z orientation


        last_pose = pose;
        last_linear_velocity = linear_velocity;
        last_angular_velocity = angular_velocity;

        return imu_msg;
    }
};

}
