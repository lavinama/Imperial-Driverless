#pragma once

#include "pose_2d.hpp"

namespace lightweight_lidar_only_simulator {

class MotionModel {
public:
    virtual Pose2D get_pose() = 0;

    virtual void set_pose(const Pose2D pose) = 0;

    virtual double get_linear_velocity() = 0;

    virtual double get_angular_velocity() = 0;

    virtual void update(const double steering_angle_rad, const double torque_nm, const double brakes_pct, const double dt) = 0;

    virtual void get_wheel_velocities(double (&return_array)[4]) = 0;
};

}
