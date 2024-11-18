#ifndef TRAJECTORY_FOLLOWER_NODE__CONTROLLER_NODE_HPP_
#define TRAJECTORY_FOLLOWER_NODE__CONTROLLER_NODE_HPP_

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"
#include "rclcpp/rclcpp.hpp"
#include "tf2/utils.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "trajectory_follower_base/lateral_controller_base.hpp"
#include "trajectory_follower_base/longitudinal_controller_base.hpp"
#include "trajectory_follower_node/visibility_control.hpp"
#include "vehicle_info_util/vehicle_info_util.hpp"

#include "autoware_auto_control_msgs/msg/ackermann_control_command.hpp"
#include "autoware_auto_control_msgs/msg/longitudinal_command.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory.hpp"
#include "autoware_auto_vehicle_msgs/msg/vehicle_odometry.hpp"
#include "geometry_msgs/msg/accel_stamped.hpp"
#include "geometry_msgs/msg/accel_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "std_msgs/msg/string.hpp"
// 追加のインクルード
#include "iino_msgs/msg/state_with_name.hpp"
#include "iino_msgs/msg/pose_stamped_with_name.hpp"

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "tf2/utils.h"  // tf2のクォータニオン操作用
#include "tf2/LinearMath/Quaternion.h"  // tf2::Quaternion用

namespace autoware::motion::control
{
using trajectory_follower::LateralOutput;
using trajectory_follower::LongitudinalOutput;
namespace trajectory_follower_node
{

using autoware_adapi_v1_msgs::msg::OperationModeState;

namespace trajectory_follower = ::autoware::motion::control::trajectory_follower;

class TRAJECTORY_FOLLOWER_PUBLIC Controller : public rclcpp::Node
{
public:
  explicit Controller(const rclcpp::NodeOptions & node_options);
  virtual ~Controller() {}

private:
  // 既存のメンバー変数
  rclcpp::TimerBase::SharedPtr timer_control_;
  double timeout_thr_sec_;
  boost::optional<LongitudinalOutput> longitudinal_output_{boost::none};

  std::shared_ptr<trajectory_follower::LongitudinalControllerBase> longitudinal_controller_;
  std::shared_ptr<trajectory_follower::LateralControllerBase> lateral_controller_;

  // 既存のSubscriber/Publisher
  rclcpp::Subscription<autoware_auto_planning_msgs::msg::Trajectory>::SharedPtr sub_ref_path_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odometry_;
  rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::SteeringReport>::SharedPtr sub_steering_;
  rclcpp::Subscription<geometry_msgs::msg::AccelWithCovarianceStamped>::SharedPtr sub_accel_;
  rclcpp::Subscription<OperationModeState>::SharedPtr sub_operation_mode_;
  rclcpp::Publisher<autoware_auto_control_msgs::msg::AckermannControlCommand>::SharedPtr control_cmd_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_marker_pub_;

  // データバッファ
  autoware_auto_planning_msgs::msg::Trajectory::SharedPtr current_trajectory_ptr_;
  nav_msgs::msg::Odometry::SharedPtr current_odometry_ptr_;
  autoware_auto_vehicle_msgs::msg::SteeringReport::SharedPtr current_steering_ptr_;
  geometry_msgs::msg::AccelWithCovarianceStamped::SharedPtr current_accel_ptr_;
  OperationModeState::SharedPtr current_operation_mode_ptr_;

  // 追加のSubscriber/Publisher
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_turn_pose_cmd_;
  rclcpp::Subscription<iino_msgs::msg::PoseStampedWithName>::SharedPtr sub_init_pose_;  // 型を変更
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_turn_pose_res_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_driving_direction_;
  bool is_reverse_mode_{false};

  // 既存の列挙型
  enum class LateralControllerMode {
    INVALID = 0,
    MPC = 1,
    PURE_PURSUIT = 2,
  };
  enum class LongitudinalControllerMode {
    INVALID = 0,
    PID = 1,
  };

  // 既存の関数
  boost::optional<trajectory_follower::InputData> createInputData(rclcpp::Clock & clock) const;
  void callbackTimerControl();
  void onTrajectory(const autoware_auto_planning_msgs::msg::Trajectory::SharedPtr);
  void onOdometry(const nav_msgs::msg::Odometry::SharedPtr msg);
  void onSteering(const autoware_auto_vehicle_msgs::msg::SteeringReport::SharedPtr msg);
  void onAccel(const geometry_msgs::msg::AccelWithCovarianceStamped::SharedPtr msg);
  bool isTimeOut(const LongitudinalOutput & lon_out, const LateralOutput & lat_out);
  LateralControllerMode getLateralControllerMode(const std::string & algorithm_name) const;
  LongitudinalControllerMode getLongitudinalControllerMode(const std::string & algorithm_name) const;
  void publishDebugMarker(
    const trajectory_follower::InputData & input_data,
    const trajectory_follower::LateralOutput & lat_out) const;

  // 追加のコールバック関数
  void onTurnPoseCmd(const std_msgs::msg::String::SharedPtr msg);
  void onInitPose(const iino_msgs::msg::PoseStampedWithName::SharedPtr msg);  // 型を変更
};

}  // namespace trajectory_follower_node
}  // namespace autoware::motion::control

#endif  // TRAJECTORY_FOLLOWER_NODE__CONTROLLER_NODE_HPP_