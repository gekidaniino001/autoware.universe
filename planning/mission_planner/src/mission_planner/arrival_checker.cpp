// Copyright 2022 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "arrival_checker.hpp"

#include <tier4_autoware_utils/tier4_autoware_utils.hpp>

#include <tf2/utils.h>
#include <iostream>
#include <iterator>
#include <string>

using std::cin;
using std::cout;
using std::endl;
using std::string;

namespace mission_planner
{

ArrivalChecker::ArrivalChecker(rclcpp::Node * node) : vehicle_stop_checker_(node)
{
  const double angle_deg = node->declare_parameter<double>("arrival_check_angle_deg");
  default_angle_ = tier4_autoware_utils::deg2rad(angle_deg);
  angle_ = default_angle_;
  default_distance_ = node->declare_parameter<double>("arrival_check_distance");
  distance_ = default_distance_;
  default_duration_ = node->declare_parameter<double>("arrival_check_duration");
  duration_ = default_duration_;
  can_change_params_ = node->declare_parameter<bool>("can_change_params", true);
  receiving_topic_ = false;
  // force_arrival_ = false;

  sub_angle_deg_ = node->create_subscription<std_msgs::msg::Float64>(
    "input/arrival_check_angle", 1, [this](const std_msgs::msg::Float64::ConstSharedPtr msg) {
      set_angle(msg->data);
    });
  
  sub_distance_ = node->create_subscription<std_msgs::msg::Float64>(
    "input/arrival_check_distance", 1,
    [this](const std_msgs::msg::Float64::ConstSharedPtr msg) { set_distance(msg->data);});
  
  sub_duration_ = node->create_subscription<std_msgs::msg::Float64>(
    "input/arrival_check_duration", 1,
    [this](const std_msgs::msg::Float64::ConstSharedPtr msg) { set_duration(msg->data); });

  // sub_force_arrival_ = node->create_subscription<std_msgs::msg::Bool>(
  //   "input/force_arrival", 1,
  //   [this](const std_msgs::msg::Bool::ConstSharedPtr msg) { force_arrival(msg->data); });

  sub_goal_ = node->create_subscription<PoseWithUuidStamped>(
    "input/modified_goal", 1,
    [this](const PoseWithUuidStamped::ConstSharedPtr msg) { modify_goal(*msg); });

  pub_unmet_goal_reason_ = node->create_publisher<std_msgs::msg::String>("debug/unmet_goal_reason", 1);
  pub_goal_distance_ = node->create_publisher<std_msgs::msg::Float64>("debug/goal_distance", 1);
  pub_arrival_distance_ = node->create_publisher<std_msgs::msg::Float64>("debug/arrival_distance", 1);

  tmr_pub_ = node->create_wall_timer(
    std::chrono::milliseconds(100), [this]() { publish_debug_info(); });
  tmr_check_receiving_topic_ = node->create_wall_timer(
    std::chrono::milliseconds(5000), [this]() { check_receiving_topic(); });
}

void ArrivalChecker::set_goal()
{
  // Ignore the modified goal after the route is cleared.
  goal_with_uuid_ = std::nullopt;
}

void ArrivalChecker::set_goal(const PoseWithUuidStamped & goal)
{
  // Ignore the modified goal for the previous route using uuid.
  goal_with_uuid_ = goal;
}

void ArrivalChecker::check_receiving_topic()
{
  if (!receiving_topic_) {
    distance_ = default_distance_;
    angle_ = default_angle_;
    duration_ = default_duration_;
  }
  receiving_topic_ = false;
}

void ArrivalChecker::modify_goal(const PoseWithUuidStamped & modified_goal)
{
  if (!goal_with_uuid_) {
    return;
  }
  if (goal_with_uuid_.value().uuid.uuid != modified_goal.uuid.uuid) {
    return;
  }
  set_goal(modified_goal);
}

void ArrivalChecker::set_distance(double distance)
{
  receiving_topic_ = true;
  if (can_change_params_ && distance >= 0)
  {
    distance_ = distance;
  } else {
    distance_ = default_distance_;
  }
}

void ArrivalChecker::set_angle(double angle)
{
  receiving_topic_ = true;
  if (can_change_params_ && angle >= 0)
  {
    angle_ = tier4_autoware_utils::deg2rad(angle);
  } else {
    angle_ = default_angle_;
  }
}

void ArrivalChecker::set_duration(double duration)
{
  receiving_topic_ = true;
  if (can_change_params_ && duration >= 0)
  {
    duration_ = duration;
  } else {
    duration_ = default_duration_;
  }
}

// void ArrivalChecker::force_arrival(bool force_arrival)
// {
//   if ( can_change_params_ && force_arrival ) {
//     force_arrival_ = true;
//   }
// }

void ArrivalChecker::publish_debug_info()
{
  pub_unmet_goal_reason_->publish(msg_unmet_goal_reason_);
  pub_goal_distance_->publish(msg_goal_distance_);
  pub_arrival_distance_->publish(msg_arrival_distance_);
}

bool ArrivalChecker::is_arrived(const PoseStamped & pose)
{
  bool has_reached_goal_ = true;
  msg_unmet_goal_reason_.data = "";
  if (!goal_with_uuid_) {
    msg_unmet_goal_reason_.data += "not_goal_with_uuid, ";
    has_reached_goal_ = false;
    // return false;
  }
  const auto goal = goal_with_uuid_.value();

  // Check frame id
  if (goal.header.frame_id != pose.header.frame_id) {
    has_reached_goal_ = false;
    msg_unmet_goal_reason_.data += "frame_id, ";
    // return false;
  }

  // Check distance.
  msg_goal_distance_.data = tier4_autoware_utils::calcDistance2d(pose.pose, goal.pose);
  msg_arrival_distance_.data = distance_;
  if (distance_ < tier4_autoware_utils::calcDistance2d(pose.pose, goal.pose)) {
    has_reached_goal_ = false;
    msg_unmet_goal_reason_.data += "distance, ";
    // return false;
  }

  // Check angle.
  const double yaw_pose = tf2::getYaw(pose.pose.orientation);
  const double yaw_goal = tf2::getYaw(goal.pose.orientation);
  const double yaw_diff = tier4_autoware_utils::normalizeRadian(yaw_pose - yaw_goal);
  if (angle_ < std::fabs(yaw_diff)) {
    has_reached_goal_ = false;
    msg_unmet_goal_reason_.data += "angle, ";
    // return false;
  }

  // Check vehicle stopped.
  // return vehicle_stop_checker_.isVehicleStopped(duration_);
  if (!vehicle_stop_checker_.isVehicleStopped(duration_)) {
    has_reached_goal_ = false;
    msg_unmet_goal_reason_.data += "vehicle_not_stopped, ";
  }

  // Check force arrival.
  // if (force_arrival_) {
  //   has_reached_goal_ = true;
  //   force_arrival_ = false;
  //   msg_unmet_goal_reason_.data = "None(force_arrival)";
  // }

  if (has_reached_goal_) {
    msg_unmet_goal_reason_.data = "None(reached_goal)";
  }

  return has_reached_goal_;
}

}  // namespace mission_planner
