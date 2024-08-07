// Copyright (c) 2024 Open Navigation LLC
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

#include <cmath>

#include "nav2_util/node_utils.hpp"
#include "opennav_docking/simple_charging_dock.hpp"

namespace opennav_docking
{

void SimpleChargingDock::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,  // 节点的弱指针
  const std::string & name,                                // 对接的名称
  std::shared_ptr<tf2_ros::Buffer> tf)                      // TF 缓冲区
{
  // 设置名称和 TF 缓冲区
  name_ = name;
  tf2_buffer_ = tf;

  // 初始化充电状态
  is_charging_ = false;

  // 锁定并获取生命周期节点
  node_ = parent.lock();
  if (!node_) {
    throw std::runtime_error{"Failed to lock node"};
  }

   // Optionally use battery info to check when charging, else say charging if docked// 可选：使用电池信息检查是否充电，否则当对接时假定充电
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".use_battery_status", rclcpp::ParameterValue(true));

 // Parameters for optional external detection of dock pose // 可选的外部检测对接姿态的参数配置
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".use_external_detection_pose", rclcpp::ParameterValue(false));
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".external_detection_timeout", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".external_detection_translation_x", rclcpp::ParameterValue(-0.20));
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".external_detection_translation_y", rclcpp::ParameterValue(0.0));
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".external_detection_rotation_yaw", rclcpp::ParameterValue(0.0));
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".external_detection_rotation_pitch", rclcpp::ParameterValue(1.57));
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".external_detection_rotation_roll", rclcpp::ParameterValue(-1.57));
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".filter_coef", rclcpp::ParameterValue(0.1));

   // Charging threshold from BatteryState message// 电池状态消息中的充电阈值
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".charging_threshold", rclcpp::ParameterValue(0.5));

   // Optionally determine if docked via stall detection using joint_states// 可选：通过关节状态检测是否对接的参数配置
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".use_stall_detection", rclcpp::ParameterValue(false));
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".stall_joint_names", rclcpp::PARAMETER_STRING_ARRAY);
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".stall_velocity_threshold", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".stall_effort_threshold", rclcpp::ParameterValue(1.0));

   // If not using stall detection, this is how close robot should get to pose // 如果不使用停滞检测，则机器人应接近姿态的距离
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".docking_threshold", rclcpp::ParameterValue(0.05));

   // Staging pose configuration// 预设姿态配置
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".staging_x_offset", rclcpp::ParameterValue(-0.7));
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".staging_yaw_offset", rclcpp::ParameterValue(0.0));

  // 从节点参数中获取配置参数
  node_->get_parameter(name + ".use_battery_status", use_battery_status_);
  node_->get_parameter(name + ".use_external_detection_pose", use_external_detection_pose_);
  node_->get_parameter(name + ".external_detection_timeout", external_detection_timeout_);
  node_->get_parameter(
    name + ".external_detection_translation_x", external_detection_translation_x_);
  node_->get_parameter(
    name + ".external_detection_translation_y", external_detection_translation_y_);
  double yaw, pitch, roll;
  node_->get_parameter(name + ".external_detection_rotation_yaw", yaw);
  node_->get_parameter(name + ".external_detection_rotation_pitch", pitch);
  node_->get_parameter(name + ".external_detection_rotation_roll", roll);
  external_detection_rotation_.setEuler(pitch, roll, yaw);
  node_->get_parameter(name + ".charging_threshold", charging_threshold_);
  node_->get_parameter(name + ".stall_velocity_threshold", stall_velocity_threshold_);
  node_->get_parameter(name + ".stall_effort_threshold", stall_effort_threshold_);
  node_->get_parameter(name + ".docking_threshold", docking_threshold_);
  node_->get_parameter(name + ".staging_x_offset", staging_x_offset_);
  node_->get_parameter(name + ".staging_yaw_offset", staging_yaw_offset_);

  // 设置滤波器
  double filter_coef;
  node_->get_parameter(name + ".filter_coef", filter_coef);
  filter_ = std::make_unique<PoseFilter>(filter_coef, external_detection_timeout_);

  // 如果使用电池状态来检测充电，创建订阅者
  if (use_battery_status_) {
    battery_sub_ = node_->create_subscription<sensor_msgs::msg::BatteryState>(
      "battery_state", 1,
      [this](const sensor_msgs::msg::BatteryState::SharedPtr state) {
        is_charging_ = state->current > charging_threshold_;
      });
  }

  // 如果使用外部检测姿态，创建订阅者
  if (use_external_detection_pose_) {
    dock_pose_.header.stamp = rclcpp::Time(0);
    dock_pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      "detected_dock_pose", 1,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr pose) {
        detected_dock_pose_ = *pose;
      });
  }

  // 如果使用停滞检测，创建订阅者并设置停滞参数
  bool use_stall_detection;
  node_->get_parameter(name + ".use_stall_detection", use_stall_detection);
  if (use_stall_detection) {
    is_stalled_ = false;
    node_->get_parameter(name + ".stall_joint_names", stall_joint_names_);
    if (stall_joint_names_.size() < 1) {
      RCLCPP_ERROR(node_->get_logger(), "stall_joint_names cannot be empty!");
    }
    joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 1,
      std::bind(&SimpleChargingDock::jointStateCallback, this, std::placeholders::_1));
  }

  // 创建发布者
  dock_pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("dock_pose", 1);
  filtered_dock_pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
    "filtered_dock_pose", 1);
  staging_pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("staging_pose", 1);
}

geometry_msgs::msg::PoseStamped SimpleChargingDock::getStagingPose(
  const geometry_msgs::msg::Pose & pose, const std::string & frame)
{
  // If not using detection, set the dock pose as the given dock pose estimate  // 如果不使用外部检测姿态，则将充电桩姿态设置为给定的充电桩姿态估计值
  if (!use_external_detection_pose_) {
    // This gets called at the start of docking
    // Reset our internally tracked dock pose
    // 这在对接开始时被调用
    // 重置内部跟踪的充电桩姿态
    dock_pose_.header.frame_id = frame;
    dock_pose_.pose = pose;
  }

  // Compute the staging pose with given offsets  // 计算带有给定偏移量的预设姿态
  const double yaw = tf2::getYaw(pose.orientation);

    // 创建一个 PoseStamped 对象来保存预设姿态
  geometry_msgs::msg::PoseStamped staging_pose;
  staging_pose.header.frame_id = frame;
  staging_pose.header.stamp = node_->now();
  staging_pose.pose = pose;
  staging_pose.pose.position.x += cos(yaw) * staging_x_offset_; // 计算预设姿态的位置（在 X 轴上偏移）
  staging_pose.pose.position.y += sin(yaw) * staging_x_offset_;
  tf2::Quaternion orientation; // 计算预设姿态的朝向（在偏航角上添加额外的偏移量）
  orientation.setEuler(0.0, 0.0, yaw + staging_yaw_offset_); // 设置预设姿态的朝向
  staging_pose.pose.orientation = tf2::toMsg(orientation);

  // Publish staging pose for debugging purposes
  staging_pose_pub_->publish(staging_pose);
  return staging_pose;
}


  /**
 * @brief 获取码头的精确位姿的方法，通常基于传感器
 * @param pose 码头位姿的初步估计。
 * @param frame 初步估计的参考框架。
 */
bool SimpleChargingDock::getRefinedPose(geometry_msgs::msg::PoseStamped & pose)
{
  // If using not detection, set the dock pose to the static fixed-frame version //如果不使用外部检测，设置码头的位姿为静态固定框架版本
  if (!use_external_detection_pose_) {
    // 发布码头位姿
    dock_pose_pub_->publish(pose);
    // 更新码头位姿
    dock_pose_ = pose;
    return true;
  }

  // If using detections, get current detections, transform to frame, and apply offsets // 如果使用外部检测，获取当前检测结果，转换到指定框架，并应用偏移量
  geometry_msgs::msg::PoseStamped detected = detected_dock_pose_;

  // Validate that external pose is new enough // 验证外部检测的位姿是否足够新
  auto timeout = rclcpp::Duration::from_seconds(external_detection_timeout_);
  if (node_->now() - detected.header.stamp > timeout) {
    RCLCPP_WARN(node_->get_logger(), "Lost detection or did not detect: timeout exceeded");
    return false;
  }

  // Transform detected pose into fixed frame. Note that the argument pose is the output of detection, but also acts as the initial estimate// 将检测到的位姿转换到固定框架。注意参数 pose 是检测的输出，同时也是初步估计，
  // and contains the frame_id of docking // 并包含对接的 frame_id
  if (detected.header.frame_id != pose.header.frame_id) {
    try {
      if (!tf2_buffer_->canTransform(
          pose.header.frame_id, detected.header.frame_id,
          detected.header.stamp, rclcpp::Duration::from_seconds(0.2))) 
      {
        RCLCPP_WARN(node_->get_logger(), "Failed to transform detected dock pose");
        return false;
      }
      // 进行坐标转换
      tf2_buffer_->transform(detected, detected, pose.header.frame_id);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(node_->get_logger(), "Failed to transform detected dock pose");
      return false;
    }
  }

  // 滤波检测到的位姿
  // Filter the detected pose
  detected = filter_->update(detected);
  // 发布过滤后的位姿
  filtered_dock_pose_pub_->publish(detected);

  // Rotate the just the orientation, then remove roll/pitch // 仅旋转方向，然后移除滚转/俯仰角
  geometry_msgs::msg::PoseStamped just_orientation;
  just_orientation.pose.orientation = tf2::toMsg(external_detection_rotation_);
  geometry_msgs::msg::TransformStamped transform;
  transform.transform.rotation = detected.pose.orientation;
  tf2::doTransform(just_orientation, just_orientation, transform);

  tf2::Quaternion orientation;
  orientation.setEuler(0.0, 0.0, tf2::getYaw(just_orientation.pose.orientation));
  dock_pose_.pose.orientation = tf2::toMsg(orientation);

  // Construct dock_pose_ by applying translation/rotation// 通过应用平移/旋转构造 dock_pose_
  dock_pose_.header = detected.header;
  dock_pose_.pose.position = detected.pose.position;
  const double yaw = tf2::getYaw(dock_pose_.pose.orientation);
  dock_pose_.pose.position.x += cos(yaw) * external_detection_translation_x_ -
    sin(yaw) * external_detection_translation_y_;
  dock_pose_.pose.position.y += sin(yaw) * external_detection_translation_x_ +
    cos(yaw) * external_detection_translation_y_;
  dock_pose_.pose.position.z = 0.0;

  // Publish & return dock pose for debugging purposes  // 发布并返回码头位姿用于调试
  dock_pose_pub_->publish(dock_pose_);
  pose = dock_pose_;
  return true;
}

bool SimpleChargingDock::isDocked()
{
  if (joint_state_sub_) { // 检查是否有 joint_state_sub_，即是否订阅了关节状态话题
    // 使用堵转检测
    return is_stalled_; // 如果是堵转状态，则表示已经停靠
  }

  if (dock_pose_.header.frame_id.empty()) { // 检查 dock_pose_ 的 frame_id 是否为空
     // Dock pose is not yet valid
    return false; // 如果为空，表示码头位置无效，不能判断是否停靠
  }

  // Find base pose in target frame // 在目标框架中找到底座位置
  geometry_msgs::msg::PoseStamped base_pose; // 定义一个底座位置的姿态消息
  base_pose.header.stamp = rclcpp::Time(0); // 设置时间戳为 0，表示使用最新的变换
  base_pose.header.frame_id = "base_link"; // 设置底座位置的参考框架为 base_link
  base_pose.pose.orientation.w = 1.0; // 设置底座姿态的四元数方向部分，表示无旋转
  try {
    tf2_buffer_->transform(base_pose, base_pose, dock_pose_.header.frame_id); // 将底座位置变换到码头位置的参考框架中
  } catch (const tf2::TransformException & ex) { // 捕获变换异常
    return false; // 变换失败，返回 false
  }

    // If we are close enough, pretend we are charging// 如果距离足够近，则假装正在充电
  double d = std::hypot(
    base_pose.pose.position.x - dock_pose_.pose.position.x, // 计算底座位置与码头位置在 x 方向的差值
    base_pose.pose.position.y - dock_pose_.pose.position.y); // 计算底座位置与码头位置在 y 方向的差值
  return d < docking_threshold_; // 如果距离小于停靠阈值，则表示已经停靠，返回 true
}

bool SimpleChargingDock::isCharging()
{
  return use_battery_status_ ? is_charging_ : isDocked();
}

bool SimpleChargingDock::disableCharging()
{
  return true;
}

bool SimpleChargingDock::hasStoppedCharging()
{
  return !isCharging();
}

void SimpleChargingDock::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr state)
{
  double velocity = 0.0; // 初始化关节速度总和
  double effort = 0.0;   // 初始化关节力矩总和

  // 遍历接收到的关节状态消息中的所有关节
  for (size_t i = 0; i < state->name.size(); ++i) {
    // 遍历需要监测的关节名称
    for (auto & name : stall_joint_names_) {
      // 如果当前关节名称与需要监测的关节名称匹配
      if (state->name[i] == name) {
        // 累加该关节的速度和力矩
        velocity += abs(state->velocity[i]);
        effort += abs(state->effort[i]);
      }
    }
  }

  // 计算监测关节的速度和力矩的平均值
  effort /= stall_joint_names_.size();
  velocity /= stall_joint_names_.size();

  // 根据速度和力矩阈值判断是否发生了卡滞
  // 如果速度低于卡滞速度阈值，并且力矩高于卡滞力矩阈值，则标记为卡滞状态
  is_stalled_ = (velocity < stall_velocity_threshold_) && (effort > stall_effort_threshold_);
}

}  // namespace opennav_docking

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(opennav_docking::SimpleChargingDock, opennav_docking_core::ChargingDock)
