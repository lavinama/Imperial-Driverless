#pragma once

#include "pose_2d.hpp"
#include "lightweight_lidar_only_simulator/motion_model.hpp"

namespace lightweight_lidar_only_simulator {

class AckermannKinematics : public MotionModel {
private:
    Pose2D pose;
    double velocity = 0;
    double steering_angle = 0;
    const double wheelbase;
    const double wheel_separation;
    const double wheel_radius;
    const double vehicle_mass;

    double wheel_velocities[4]; //fl, fr, rl, rr
    
public:
    AckermannKinematics(
        const Pose2D initial_pose,
        const double wheelbase, 
        const double wheel_separation, 
        const double wheel_radius,
        const double vehicle_mass
    ); 

    Pose2D get_pose();

    void set_pose(const Pose2D pose);

    double get_linear_velocity();

    double get_angular_velocity();

    void update(const double steering_angle_rad, const double torque_nm, const double brakes_pct, const double dt);

    void get_wheel_velocities(double (&return_array)[4]);
};

}
