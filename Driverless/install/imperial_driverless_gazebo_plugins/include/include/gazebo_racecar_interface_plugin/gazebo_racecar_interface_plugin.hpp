/*
 * AMZ-Driverless
 * Copyright (c) 2018 Authors:
 *   - Juraj Kabzan <kabzanj@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef XDid_plugins_GAZEBO_RACE_CAR_MODEL_INCLUDE_GAZEBO_RACE_CAR_MODEL_GAZEBO_ROS_RACE_CAR_HPP_
#define XDid_plugins_GAZEBO_RACE_CAR_MODEL_INCLUDE_GAZEBO_RACE_CAR_MODEL_GAZEBO_ROS_RACE_CAR_HPP_

#include <memory>
#include <vector>
#include <string>
#include <queue>
// ROS Includes
#include "rclcpp/rclcpp.hpp"

// ROS msgs
#include "imperial_driverless_interfaces/msg/vcu_drive_command.hpp"

// ROS TF2
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/transform_datatypes.h>
#include <tf2/utils.h>

// Gazebo Includes
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo_ros/node.hpp>

namespace gazebo_plugins {
namespace id_plugins {

class GazeboRaceCarInterfacePlugin : public gazebo::ModelPlugin {
 public:
  GazeboRaceCarInterfacePlugin();

  ~GazeboRaceCarInterfacePlugin() override;

  void Reset() override;
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override;

 private:
  void initParams(const sdf::ElementPtr &sdf);
  void initModel(const sdf::ElementPtr &sdf);
  void publishTf();
  void update();
  void updateState(double dt);
  void executeCmd(const imperial_driverless_interfaces::msg::VCUDriveCommand::SharedPtr msg);
  void onCmd(const imperial_driverless_interfaces::msg::VCUDriveCommand::SharedPtr msg);


  std::shared_ptr<rclcpp::Node> rosnode;
  
  // plugin parameters
  double update_rate;
  double publish_rate;
  std::string robot_frame;
  bool publish_tf;

  
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_br;

  // ROS parameters
  std::string ground_truth_car_state_topic;
  std::string localisation_car_state_topic;
  std::string wheel_speeds_topic_name;
  std::string ground_truth_wheel_speeds_topic_name;
  std::string odom_topic_name;
  double control_delay;

  double wheelbase;
  double wheel_separation;

  // ROS Publishers
  // rclcpp::Publisher<eufs_msgs::msg::CarState>::SharedPtr _pub_ground_truth_car_state;

  // ROS Subscriptions
  rclcpp::Subscription<imperial_driverless_interfaces::msg::VCUDriveCommand>::SharedPtr sub_cmd;

  imperial_driverless_interfaces::msg::VCUDriveCommand::SharedPtr current_cmd;

  gazebo::common::Time time_last_published;
  gazebo::common::Time last_sim_time;
  gazebo::common::Time last_cmd_time;

  gazebo::physics::WorldPtr world;
  gazebo::physics::ModelPtr model;
  gazebo::event::ConnectionPtr update_connection;

  gazebo::physics::JointPtr left_steering_joint;
  gazebo::physics::JointPtr right_steering_joint;
  gazebo::physics::JointPtr left_rear_wheel_joint;
  gazebo::physics::JointPtr left_front_wheel_joint;
  gazebo::physics::JointPtr right_rear_wheel_joint;
  gazebo::physics::JointPtr right_front_wheel_joint;
  gazebo::physics::LinkPtr base_link_;

  // Command queue for control delays
  std::queue<std::shared_ptr<imperial_driverless_interfaces::msg::VCUDriveCommand>> command_Q;
  std::queue<gazebo::common::Time> cmd_time_Q;
  // Steering rate limit variables
};

}  // namespace id_plugins
}  // namespace gazebo_plugins

#endif  // id_plugins_GAZEBO_RACE_CAR_MODEL_INCLUDE_GAZEBO_RACE_CAR_MODEL_GAZEBO_ROS_RACE_CAR_HPP_
