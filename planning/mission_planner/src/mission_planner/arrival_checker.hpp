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

#ifndef MISSION_PLANNER__ARRIVAL_CHECKER_HPP_
#define MISSION_PLANNER__ARRIVAL_CHECKER_HPP_

#include <motion_utils/vehicle/vehicle_state_checker.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_planning_msgs/msg/pose_with_uuid_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>

namespace mission_planner
{

class ArrivalChecker
{
public:
  using PoseWithUuidStamped = autoware_planning_msgs::msg::PoseWithUuidStamped;
  using PoseStamped = geometry_msgs::msg::PoseStamped;
  using Float64 = std_msgs::msg::Float64;
  using String = std_msgs::msg::String;
  explicit ArrivalChecker(rclcpp::Node * node);
  void set_goal();
  void set_goal(const PoseWithUuidStamped & goal);
  bool is_arrived(const PoseStamped & pose);

private:
  double distance_;
  double default_distance_;
  double angle_;
  double default_angle_;
  double duration_;
  double default_duration_;
  double can_change_params_;
  std::optional<PoseWithUuidStamped> goal_with_uuid_;
  std_msgs::msg::String msg_unmet_goal_reason_;
  std_msgs::msg::Float64 msg_goal_distance_;
  std_msgs::msg::Float64 msg_arrival_distance_;
  rclcpp::Subscription<Float64>::SharedPtr sub_angle_deg_;
  rclcpp::Subscription<Float64>::SharedPtr sub_distance_;
  rclcpp::Subscription<Float64>::SharedPtr sub_duration_;
  // rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_force_arrival_;
  rclcpp::Subscription<PoseWithUuidStamped>::SharedPtr sub_goal_;
  rclcpp::Publisher<String>::SharedPtr pub_unmet_goal_reason_;
  rclcpp::Publisher<Float64>::SharedPtr pub_goal_distance_;
  rclcpp::Publisher<Float64>::SharedPtr pub_arrival_distance_;
  motion_utils::VehicleStopChecker vehicle_stop_checker_;
  bool receiving_topic_;
  // bool force_arrival_;
  void check_receiving_topic();
  void modify_goal(const PoseWithUuidStamped & modified_goal);
  void set_distance(double distance);
  void set_angle(double angle);
  void set_duration(double duration);
  // void force_arrival(bool force);
  void publish_debug_info();
  rclcpp::TimerBase::SharedPtr tmr_pub_{};
  rclcpp::TimerBase::SharedPtr tmr_check_receiving_topic_{};
};

}  // namespace mission_planner

#endif  // MISSION_PLANNER__ARRIVAL_CHECKER_HPP_
