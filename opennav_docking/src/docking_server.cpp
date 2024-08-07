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

#include "angles/angles.h"
#include "opennav_docking/docking_server.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"

using namespace std::chrono_literals;
using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

namespace opennav_docking
{

DockingServer::DockingServer(const rclcpp::NodeOptions & options)
: nav2_util::LifecycleNode("docking_server", "", options)
{
  RCLCPP_INFO(get_logger(), "Creating %s", get_name());

  declare_parameter("controller_frequency", 50.0);
  declare_parameter("initial_perception_timeout", 5.0);
  declare_parameter("wait_charge_timeout", 5.0);
  declare_parameter("dock_approach_timeout", 30.0);
  declare_parameter("undock_linear_tolerance", 0.05);
  declare_parameter("undock_angular_tolerance", 0.05);
  declare_parameter("max_retries", 3);
  declare_parameter("base_frame", "base_link");
  declare_parameter("fixed_frame", "odom");
  declare_parameter("dock_backwards", false);
  declare_parameter("dock_prestaging_tolerance", 0.5);
}

nav2_util::CallbackReturn
DockingServer::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring %s", get_name()); // 打印日志，正在配置服务器
  auto node = shared_from_this();

  // 获取参数
  get_parameter("controller_frequency", controller_frequency_);
  get_parameter("initial_perception_timeout", initial_perception_timeout_);
  get_parameter("wait_charge_timeout", wait_charge_timeout_);
  get_parameter("dock_approach_timeout", dock_approach_timeout_);
  get_parameter("undock_linear_tolerance", undock_linear_tolerance_);
  get_parameter("undock_angular_tolerance", undock_angular_tolerance_);
  get_parameter("max_retries", max_retries_);
  get_parameter("base_frame", base_frame_);
  get_parameter("fixed_frame", fixed_frame_);
  get_parameter("dock_backwards", dock_backwards_);
  get_parameter("dock_prestaging_tolerance", dock_prestaging_tolerance_);
  RCLCPP_INFO(get_logger(), "Controller frequency set to %.4fHz", controller_frequency_); // 打印控制器频率

  // 创建速度发布者
  vel_publisher_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
  tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock()); // 创建tf2缓存

  // 声明并获取参数，如果参数未声明则使用默认值
  double action_server_result_timeout;
  nav2_util::declare_parameter_if_not_declared(
    node, "action_server_result_timeout", rclcpp::ParameterValue(10.0));
  get_parameter("action_server_result_timeout", action_server_result_timeout);
  rcl_action_server_options_t server_options = rcl_action_server_get_default_options();
  server_options.result_timeout.nanoseconds = RCL_S_TO_NS(action_server_result_timeout);

  // 创建对接/解除对接的动作服务器
  docking_action_server_ = std::make_unique<DockingActionServer>(
    node, "dock_robot",
    std::bind(&DockingServer::dockRobot, this),
    nullptr, std::chrono::milliseconds(500),
    true, server_options);

  undocking_action_server_ = std::make_unique<UndockingActionServer>(
    node, "undock_robot",
    std::bind(&DockingServer::undockRobot, this),
    nullptr, std::chrono::milliseconds(500),
    true, server_options);

  // 创建组合工具
  controller_ = std::make_unique<Controller>(node);
  navigator_ = std::make_unique<Navigator>(node);
  dock_db_ = std::make_unique<DockDatabase>();
  if (!dock_db_->initialize(node, tf2_buffer_)) { // 初始化数据库，若失败返回失败状态
    return nav2_util::CallbackReturn::FAILURE;
  }

  return nav2_util::CallbackReturn::SUCCESS; // 成功配置返回成功状态
}

nav2_util::CallbackReturn
DockingServer::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating %s", get_name());

  auto node = shared_from_this();

  tf2_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf2_buffer_);
  dock_db_->activate();
  navigator_->activate();
  vel_publisher_->on_activate();
  docking_action_server_->activate();
  undocking_action_server_->activate();
  curr_dock_type_.clear();

  // Add callback for dynamic parameters
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&DockingServer::dynamicParametersCallback, this, _1));

  // Create bond connection
  createBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
DockingServer::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating %s", get_name());

  docking_action_server_->deactivate();
  undocking_action_server_->deactivate();
  dock_db_->deactivate();
  navigator_->deactivate();
  vel_publisher_->on_deactivate();

  dyn_params_handler_.reset();
  tf2_listener_.reset();

  // Destroy bond connection
  destroyBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
DockingServer::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up %s", get_name());
  tf2_buffer_.reset();
  docking_action_server_.reset();
  undocking_action_server_.reset();
  dock_db_.reset();
  navigator_.reset();
  curr_dock_type_.clear();
  controller_.reset();
  vel_publisher_.reset();
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
DockingServer::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shutting down %s", get_name());
  return nav2_util::CallbackReturn::SUCCESS;
}

template<typename ActionT>
void DockingServer::getPreemptedGoalIfRequested(
  typename std::shared_ptr<const typename ActionT::Goal> goal,
  const std::unique_ptr<nav2_util::SimpleActionServer<ActionT>> & action_server)
{
  if (action_server->is_preempt_requested()) {
    goal = action_server->accept_pending_goal();
  }
}

template<typename ActionT>
bool DockingServer::checkAndWarnIfCancelled(
  std::unique_ptr<nav2_util::SimpleActionServer<ActionT>> & action_server,
  const std::string & name)
{
  if (action_server->is_cancel_requested()) {
    RCLCPP_WARN(get_logger(), "Goal was cancelled. Cancelling %s action", name.c_str());
    return true;
  }
  return false;
}

template<typename ActionT>
bool DockingServer::checkAndWarnIfPreempted(
  std::unique_ptr<nav2_util::SimpleActionServer<ActionT>> & action_server,
  const std::string & name)
{
  if (action_server->is_preempt_requested()) {
    RCLCPP_WARN(get_logger(), "Goal was preempted. Cancelling %s action", name.c_str());
    return true;
  }
  return false;
}

void DockingServer::dockRobot()
{
  // 锁定动态参数互斥量，保证线程安全
  std::lock_guard<std::mutex> lock(dynamic_params_lock_);
  // 记录动作开始时间
  action_start_time_ = this->now();
  // 根据控制频率设置循环速率
  rclcpp::Rate loop_rate(controller_frequency_);

  // 获取当前目标和初始化结果对象
  auto goal = docking_action_server_->get_current_goal();
  auto result = std::make_shared<DockRobot::Result>();
  result->success = false;

  // 检查动作服务器是否可用和活动
  if (!docking_action_server_ || !docking_action_server_->is_server_active()) {
    RCLCPP_DEBUG(get_logger(), "Action server unavailable or inactive. Stopping.");
    return;
  }

  // 检查并警告是否取消动作
  if (checkAndWarnIfCancelled(docking_action_server_, "dock_robot")) {
    docking_action_server_->terminate_all();
    return;
  }

  // 如果需要抢占目标，获取抢占目标
  getPreemptedGoalIfRequested(goal, docking_action_server_);
  Dock * dock{nullptr};
  num_retries_ = 0;

  try {
    // 根据请求获取码头实例和插件信息
     // Get dock (instance and plugin information) from request
    if (goal->use_dock_id) {
      RCLCPP_INFO(
        get_logger(),
        "Attempting to dock robot at charger %s.", goal->dock_id.c_str());
      dock = dock_db_->findDock(goal->dock_id);
    } else {
      RCLCPP_INFO(
        get_logger(),
        "Attempting to dock robot at charger at position (%0.2f, %0.2f).",
        goal->dock_pose.pose.position.x, goal->dock_pose.pose.position.y);
      dock = generateGoalDock(goal);
    }

    // 发送机器人到其预定位姿
    // Send robot to its staging pose
    publishDockingFeedback(DockRobot::Feedback::NAV_TO_STAGING_POSE);
    const auto initial_staging_pose = dock->getStagingPose();
    const auto robot_pose = getRobotPoseInFrame(
      initial_staging_pose.header.frame_id);
      
    if (!goal->navigate_to_staging_pose ||
      utils::l2Norm(robot_pose.pose, initial_staging_pose.pose) < dock_prestaging_tolerance_)
    {
      RCLCPP_INFO(get_logger(), "Robot already within pre-staging pose tolerance for dock");
    } else {
      navigator_->goToPose(
        initial_staging_pose, rclcpp::Duration::from_seconds(goal->max_staging_time));
      RCLCPP_INFO(get_logger(), "Successful navigation to staging pose");
    }

    // 构建码头在固定帧中的初始位置估计
    // Construct initial estimate of where the dock is located in fixed_frame
    auto dock_pose = utils::getDockPoseStamped(dock, rclcpp::Time(0));
    tf2_buffer_->transform(dock_pose, dock_pose, fixed_frame_);

    // 在继续移动前获取码头的初始检测
     // Get initial detection of dock before proceeding to move
    doInitialPerception(dock, dock_pose);
    RCLCPP_INFO(get_logger(), "Successful initial dock detection");

    // 停靠控制循环：如果未停靠，运行控制器
     // Docking control loop: while not docked, run controller
    rclcpp::Time dock_contact_time;
    while (rclcpp::ok()) {
      try {
        // 使用控制律靠近码头
        // Approach the dock using control law
        if (approachDock(dock, dock_pose)) {
          // 机器人已停靠，等待充电开始
          // We are docked, wait for charging to begin
          RCLCPP_INFO(get_logger(), "Made contact with dock, waiting for charge to start");
          if (waitForCharge(dock)) {
            RCLCPP_INFO(get_logger(), "Robot is charging!");
            result->success = true;
            result->num_retries = num_retries_;
            stashDockData(goal->use_dock_id, dock, true);
            publishZeroVelocity();
            docking_action_server_->succeeded_current(result);
            return;
          }
        }

        // 取消、中止或关闭（可恢复错误抛出DockingException）
        // Cancelled, preempted, or shutting down (recoverable errors throw DockingException)
        stashDockData(goal->use_dock_id, dock, false);
        publishZeroVelocity();
        docking_action_server_->terminate_all(result);
        return;
      } catch (opennav_docking_core::DockingException & e) {
        if (++num_retries_ > max_retries_) {
          RCLCPP_ERROR(get_logger(), "Failed to dock, all retries have been used");
          throw;
        }
        RCLCPP_WARN(get_logger(), "Docking failed, will retry: %s", e.what());
      }

      // 重置到预定位姿重新尝试
       // Reset to staging pose to try again
      if (!resetApproach(dock->getStagingPose())) {
        // 取消、中止或关闭
        // Cancelled, preempted, or shutting down
        stashDockData(goal->use_dock_id, dock, false);
        publishZeroVelocity();
        docking_action_server_->terminate_all(result);
        return;
      }
      RCLCPP_INFO(get_logger(), "Returned to staging pose, attempting docking again");
    }
  } catch (const tf2::TransformException & e) {
    RCLCPP_ERROR(get_logger(), "Transform error: %s", e.what());
    result->error_code = DockRobot::Result::UNKNOWN;
  } catch (opennav_docking_core::DockNotInDB & e) {
    RCLCPP_ERROR(get_logger(), "%s", e.what());
    result->error_code = DockRobot::Result::DOCK_NOT_IN_DB;
  } catch (opennav_docking_core::DockNotValid & e) {
    RCLCPP_ERROR(get_logger(), "%s", e.what());
    result->error_code = DockRobot::Result::DOCK_NOT_VALID;
  } catch (opennav_docking_core::FailedToStage & e) {
    RCLCPP_ERROR(get_logger(), "%s", e.what());
    result->error_code = DockRobot::Result::FAILED_TO_STAGE;
  } catch (opennav_docking_core::FailedToDetectDock & e) {
    RCLCPP_ERROR(get_logger(), "%s", e.what());
    result->error_code = DockRobot::Result::FAILED_TO_DETECT_DOCK;
  } catch (opennav_docking_core::FailedToControl & e) {
    RCLCPP_ERROR(get_logger(), "%s", e.what());
    result->error_code = DockRobot::Result::FAILED_TO_CONTROL;
  } catch (opennav_docking_core::FailedToCharge & e) {
    RCLCPP_ERROR(get_logger(), "%s", e.what());
    result->error_code = DockRobot::Result::FAILED_TO_CHARGE;
  } catch (opennav_docking_core::DockingException & e) {
    RCLCPP_ERROR(get_logger(), "%s", e.what());
    result->error_code = DockRobot::Result::UNKNOWN;
  } catch (std::exception & e) {
    RCLCPP_ERROR(get_logger(), "%s", e.what());
    result->error_code = DockRobot::Result::UNKNOWN;
  }

  // 存储码头状态以供以后解停靠，并删除临时码头（如果适用）
  // Store dock state for later undocking and delete temp dock, if applicable
  stashDockData(goal->use_dock_id, dock, false);
  result->num_retries = num_retries_;
  publishZeroVelocity();
  docking_action_server_->terminate_current(result);
}

void DockingServer::stashDockData(bool use_dock_id, Dock * dock, bool successful)
{
  if (dock && successful) {
    curr_dock_type_ = dock->type;
  }

  if (!use_dock_id && dock) {
    delete dock;
    dock = nullptr;
  }
}

Dock * DockingServer::generateGoalDock(std::shared_ptr<const DockRobot::Goal> goal)
{
  auto dock = new Dock();
  dock->frame = goal->dock_pose.header.frame_id;
  dock->pose = goal->dock_pose.pose;
  dock->type = goal->dock_type;
  dock->plugin = dock_db_->findDockPlugin(dock->type);
  return dock;
}

void DockingServer::doInitialPerception(Dock * dock, geometry_msgs::msg::PoseStamped & dock_pose)
{
  // 发布反馈信息，表示进入初始感知状态
  publishDockingFeedback(DockRobot::Feedback::INITIAL_PERCEPTION);

  // 设置循环频率
  rclcpp::Rate loop_rate(controller_frequency_);

  // 获取当前时间，作为开始时间
  auto start = this->now();

  // 设置超时时间
  auto timeout = rclcpp::Duration::from_seconds(initial_perception_timeout_);

  // 循环执行，直到感知到精确的充电桩姿态或者超时
  while (!dock->plugin->getRefinedPose(dock_pose)) {
    // 检查是否超时，如果超时则抛出异常
    if (this->now() - start > timeout) {
      throw opennav_docking_core::FailedToDetectDock("Failed initial dock detection");
    }

    // 检查任务是否被取消或抢占，如果是，则退出
    if (checkAndWarnIfCancelled(docking_action_server_, "dock_robot") ||
      checkAndWarnIfPreempted(docking_action_server_, "dock_robot"))
    {
      return;
    }

    // 按照设置的循环频率休眠一段时间
    loop_rate.sleep();
  }
}

bool DockingServer::approachDock(Dock * dock, geometry_msgs::msg::PoseStamped & dock_pose)
{
  // 设置控制频率
  rclcpp::Rate loop_rate(controller_frequency_);
  // 获取当前时间作为开始时间
  auto start = this->now();
  // 设置超时时间
  auto timeout = rclcpp::Duration::from_seconds(dock_approach_timeout_);
  
  // 进入循环，持续尝试靠近充电桩
  while (rclcpp::ok()) {
    // 发布控制反馈，表示正在控制
    publishDockingFeedback(DockRobot::Feedback::CONTROLLING);

    // 如果已经连接到充电桩或者正在充电，停止并报告成功
    // Stop and report success if connected to dock
    if (dock->plugin->isDocked() || dock->plugin->isCharging()) {
      return true;
    }

    // 如果操作被取消或抢占，停止并报告失败
    // Stop if cancelled/preempted
    if (checkAndWarnIfCancelled(docking_action_server_, "dock_robot") ||
      checkAndWarnIfPreempted(docking_action_server_, "dock_robot"))
    {
      return false;
    }

    // 更新感知信息，如果无法获取精确的位置信息，则抛出异常
    // Update perception
    if (!dock->plugin->getRefinedPose(dock_pose)) {
      throw opennav_docking_core::FailedToDetectDock("Failed dock detection");
    }

    // 将目标位置变换到 base_link 坐标系中
     // Transform target_pose into base_link frame
    geometry_msgs::msg::PoseStamped target_pose = dock_pose;
    target_pose.header.stamp = rclcpp::Time(0);

    // 控制算法在接近终点时可能会出现抖动，因此我们将控制目标位置向后投影一段距离
    // 以确保机器人在接触充电桩前不会到达螺旋轨迹的终点，从而停止对接过程。
    // The control law can get jittery when close to the end when atan2's can explode.
    // Thus, we backward project the controller's target pose a little bit after the
    // dock so that the robot never gets to the end of the spiral before its in contact
    // with the dock to stop the docking procedure.
    const double backward_projection = 0.25;  // 向后投影的距离
    const double yaw = tf2::getYaw(target_pose.pose.orientation);  // 获取目标方向的偏航角
    target_pose.pose.position.x += cos(yaw) * backward_projection;  // 计算新的x位置
    target_pose.pose.position.y += sin(yaw) * backward_projection;  // 计算新的y位置
    tf2_buffer_->transform(target_pose, target_pose, base_frame_);  // 变换到 base_link 坐标系

    // 计算并发布控制命令
     // Compute and publish controls
    geometry_msgs::msg::Twist command;
    if (!controller_->computeVelocityCommand(target_pose.pose, command, dock_backwards_)) {
      throw opennav_docking_core::FailedToControl("Failed to get control");
    }
    vel_publisher_->publish(command);  // 发布速度指令

    // 如果超过超时时间，抛出异常
    if (this->now() - start > timeout) {
      throw opennav_docking_core::FailedToControl(
              "Timed out approaching dock; dock nor charging detected");
    }

    // 按照设定的频率休眠
    loop_rate.sleep();
  }
  return false;  // 如果循环中断，返回失败
}

bool DockingServer::waitForCharge(Dock * dock)
{
  // 创建一个Rate对象，用于控制循环的频率，频率由controller_frequency_决定
  rclcpp::Rate loop_rate(controller_frequency_);

  // 获取当前时间
  auto start = this->now();

  // 设置等待充电的超时时间，单位是秒
  auto timeout = rclcpp::Duration::from_seconds(wait_charge_timeout_);

  // 循环直到满足某个条件或者发生错误
  while (rclcpp::ok()) {
    // 发布充电反馈信息，表明机器人正在等待充电
    publishDockingFeedback(DockRobot::Feedback::WAIT_FOR_CHARGE);

    // 检查Dock对象中的插件是否正在充电
    if (dock->plugin->isCharging()) {
      // 如果充电开始，返回true，表示等待充电成功
      return true;
    }

    // 检查是否被取消（docking_action_server_的取消请求）或被抢占（docking_action_server_的抢占请求）
    // 如果发现取消或抢占请求，返回false，表示等待充电被取消或中断
    if (checkAndWarnIfCancelled(docking_action_server_, "dock_robot") ||
      checkAndWarnIfPreempted(docking_action_server_, "dock_robot"))
    {
      return false;
    }

    // 检查从开始到现在是否超过了超时时间
    if (this->now() - start > timeout) {
      // 如果超时，抛出异常，表明等待充电超时
      throw opennav_docking_core::FailedToCharge("Timed out waiting for charge to start");
    }

    // 按照loop_rate指定的频率休眠，防止CPU占用过高
    loop_rate.sleep();
  }

  // 如果循环退出，返回false，表示未能成功等待充电（这通常不会发生，因为rclcpp::ok()应该保持为true）
  return false;
}

bool DockingServer::resetApproach(const geometry_msgs::msg::PoseStamped & staging_pose)
{
  // 设置循环频率
  rclcpp::Rate loop_rate(controller_frequency_);

  // 获取当前时间，作为开始时间
  auto start = this->now();

  // 设置超时时间
  auto timeout = rclcpp::Duration::from_seconds(dock_approach_timeout_);

  // 如果系统正常运行，进入循环
  while (rclcpp::ok()) {
    // 发布反馈信息，表示进入初始感知状态
    publishDockingFeedback(DockRobot::Feedback::INITIAL_PERCEPTION);

    // 如果任务被取消或抢占，停止操作并返回 false
    if (checkAndWarnIfCancelled(docking_action_server_, "dock_robot") ||
      checkAndWarnIfPreempted(docking_action_server_, "dock_robot"))
    {
      return false;
    }

    // 计算并发布速度命令
    geometry_msgs::msg::Twist command;
    if (getCommandToPose(
        command, staging_pose, undock_linear_tolerance_, undock_angular_tolerance_,
        !dock_backwards_)) // 计算从当前位置到预设位置的速度命令
    {
      return true; // 如果已经到达预设位置，返回 true
    }
    // 发布速度命令
    vel_publisher_->publish(command);

    // 检查是否超过超时时间
    if (this->now() - start > timeout) {
      // 如果超时，抛出异常
      throw opennav_docking_core::FailedToControl("Timed out resetting dock approach");
    }

    // 按照设置的循环频率休眠一段时间
    loop_rate.sleep();
  }

  // 如果循环退出，返回 false
  return false;
}

/**
 * @brief 
 * 
 * @param cmd 
 * @param pose 
 * @param linear_tolerance 
 * @param angular_tolerance 
 * @param backward 
 * @return true 
 * @return false 
 */
bool DockingServer::getCommandToPose(
  geometry_msgs::msg::Twist & cmd, const geometry_msgs::msg::PoseStamped & pose,
  double linear_tolerance, double angular_tolerance, bool backward)
{
  // 重置命令为零速度
  // Reset command to zero velocity
  cmd.linear.x = 0;
  cmd.angular.z = 0;

  // 判断是否已经到达目标姿态并停止
  // Determine if we have reached pose yet & stop
  geometry_msgs::msg::PoseStamped robot_pose = getRobotPoseInFrame(pose.header.frame_id); // 获取机器人当前姿态
  const double dist = std::hypot(
    robot_pose.pose.position.x - pose.pose.position.x,
    robot_pose.pose.position.y - pose.pose.position.y); // 计算位置距离
  const double yaw = angles::shortest_angular_distance(
    tf2::getYaw(robot_pose.pose.orientation), tf2::getYaw(pose.pose.orientation)); // 计算角度距离

  if (dist < linear_tolerance && abs(yaw) < angular_tolerance) { // 如果距离和角度均在容差范围内
    return true; // 返回 true 表示已经到达目标姿态
  }

  // 将目标姿态转换为 base_link 坐标系
   // Transform target_pose into base_link frame
  geometry_msgs::msg::PoseStamped target_pose = pose;
  target_pose.header.stamp = rclcpp::Time(0);
  tf2_buffer_->transform(target_pose, target_pose, base_frame_); // 使用 tf2_buffer_ 进行转换

  // 计算速度命令
  // Compute velocity command
  if (!controller_->computeVelocityCommand(target_pose.pose, cmd, backward)) { // 调用控制器计算速度命令
    throw opennav_docking_core::FailedToControl("Failed to get control"); // 失败则抛出异常
  }

  // 命令有效，但目标未到达
  // Command is valid, but target is not reached
  return false; // 返回 false 表示尚未到达目标姿态
}

void DockingServer::undockRobot()
{
  std::lock_guard<std::mutex> lock(dynamic_params_lock_); // 锁定以确保参数的线程安全
  action_start_time_ = this->now(); // 记录动作开始的时间
  rclcpp::Rate loop_rate(controller_frequency_); // 根据控制器频率设置循环速率

  auto goal = undocking_action_server_->get_current_goal(); // 获取当前目标
  auto result = std::make_shared<UndockRobot::Result>();
  result->success = false; // 初始化结果为失败

  // 检查动作服务器是否可用或活跃
  if (!undocking_action_server_ || !undocking_action_server_->is_server_active()) {
    RCLCPP_DEBUG(get_logger(), "Action server unavailable or inactive. Stopping.");
    return;
  }

  // 检查并警告是否取消
  if (checkAndWarnIfCancelled(undocking_action_server_, "undock_robot")) {
    undocking_action_server_->terminate_all(result); // 终止所有动作
    return;
  }

  // 如果请求了抢占目标，则获取该目标
  getPreemptedGoalIfRequested(goal, undocking_action_server_);
  auto max_duration = rclcpp::Duration::from_seconds(goal->max_undocking_time); // 最大脱离时间

  try {
    // 从请求或停靠状态获取停靠插件信息，重置状态
    // Get dock plugin information from request or docked state, reset state.
    std::string dock_type = curr_dock_type_;
    if (!goal->dock_type.empty()) {
      dock_type = goal->dock_type;
    }

    ChargingDock::Ptr dock = dock_db_->findDockPlugin(dock_type); // 查找停靠插件
    if (!dock) {
      throw opennav_docking_core::DockNotValid("No dock information to undock from!"); // 如果未找到插件，抛出异常
    }
    RCLCPP_INFO(
      get_logger(),
      "Attempting to undock robot from charger of type %s.", dock->getName().c_str());

    // 通过找到机器人姿态获取“停靠姿态”
    // Get "dock pose" by finding the robot pose
    geometry_msgs::msg::PoseStamped dock_pose = getRobotPoseInFrame(fixed_frame_);

    // 获取预设姿态（在固定框架中）
    // Get staging pose (in fixed frame)
    geometry_msgs::msg::PoseStamped staging_pose =
      dock->getStagingPose(dock_pose.pose, dock_pose.header.frame_id);

    // 控制机器人到预设姿态
    // Control robot to staging pose
    rclcpp::Time loop_start = this->now(); // 记录循环开始时间
    while (rclcpp::ok()) {
      // 如果超过最大持续时间则停止
      // Stop if we exceed max duration
      auto timeout = rclcpp::Duration::from_seconds(goal->max_undocking_time);
      if (this->now() - loop_start > timeout) {
        throw opennav_docking_core::FailedToControl("Undocking timed out"); // 抛出超时异常
      }

      // 如果取消或抢占则停止
       // Stop if cancelled/preempted
      if (checkAndWarnIfCancelled(undocking_action_server_, "undock_robot") ||
        checkAndWarnIfPreempted(undocking_action_server_, "undock_robot"))
      {
        publishZeroVelocity(); // 发布零速度
        undocking_action_server_->terminate_all(result); // 终止所有动作
        return;
      }

      // 在充电未禁用前不控制机器人
      // Don't control the robot until charging is disabled
      if (!dock->disableCharging()) {
        loop_rate.sleep(); // 睡眠一段时间再继续
        continue;
      }

      // 获取接近预设姿态的命令
      // Get command to approach staging pose
      geometry_msgs::msg::Twist command;
      if (getCommandToPose(
          command, staging_pose, undock_linear_tolerance_, undock_angular_tolerance_,
          !dock_backwards_))
      {
        RCLCPP_INFO(get_logger(), "Robot has reached staging pose"); // 机器人已到达预设姿态
        // Have reached staging_pose
        vel_publisher_->publish(command);
        if (dock->hasStoppedCharging()) {
          RCLCPP_INFO(get_logger(), "Robot has undocked!"); // 机器人已脱离充电器
          result->success = true;
          curr_dock_type_.clear(); // 清除当前停靠类型
          publishZeroVelocity();
          undocking_action_server_->succeeded_current(result); // 标记当前动作成功
          return;
        }
        // 未停止充电则抛出异常
        // Haven't stopped charging?
        throw opennav_docking_core::FailedToControl("Failed to control off dock, still charging");
      }

      // 发布命令并睡眠
       // Publish command and sleep
      vel_publisher_->publish(command);
      loop_rate.sleep();
    }
  } catch (const tf2::TransformException & e) {
    RCLCPP_ERROR(get_logger(), "Transform error: %s", e.what());
    result->error_code = DockRobot::Result::UNKNOWN; // 变换异常
  } catch (opennav_docking_core::DockNotValid & e) {
    RCLCPP_ERROR(get_logger(), "%s", e.what());
    result->error_code = DockRobot::Result::DOCK_NOT_VALID; // 停靠无效
  } catch (opennav_docking_core::FailedToControl & e) {
    RCLCPP_ERROR(get_logger(), "%s", e.what());
    result->error_code = DockRobot::Result::FAILED_TO_CONTROL; // 控制失败
  } catch (opennav_docking_core::DockingException & e) {
    RCLCPP_ERROR(get_logger(), "%s", e.what());
    result->error_code = DockRobot::Result::UNKNOWN; // 停靠异常
  } catch (std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Internal error: %s", e.what());
    result->error_code = DockRobot::Result::UNKNOWN; // 内部错误
  }

  publishZeroVelocity(); // 发布零速度
  undocking_action_server_->terminate_current(result); // 终止当前动作
}

geometry_msgs::msg::PoseStamped DockingServer::getRobotPoseInFrame(const std::string & frame)
{
  geometry_msgs::msg::PoseStamped robot_pose;
  robot_pose.header.frame_id = base_frame_;
  robot_pose.header.stamp = rclcpp::Time(0);
  tf2_buffer_->transform(robot_pose, robot_pose, frame);
  return robot_pose;
}

void DockingServer::publishZeroVelocity()
{
  vel_publisher_->publish(geometry_msgs::msg::Twist());
}

void DockingServer::publishDockingFeedback(uint16_t state)
{
  auto feedback = std::make_shared<DockRobot::Feedback>();
  feedback->state = state;
  feedback->docking_time = this->now() - action_start_time_;
  feedback->num_retries = num_retries_;
  docking_action_server_->publish_feedback(feedback);
}

rcl_interfaces::msg::SetParametersResult
DockingServer::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
  std::lock_guard<std::mutex> lock(dynamic_params_lock_);

  rcl_interfaces::msg::SetParametersResult result;
  for (auto parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    if (type == ParameterType::PARAMETER_DOUBLE) {
      if (name == "controller_frequency") {
        controller_frequency_ = parameter.as_double();
      } else if (name == "initial_perception_timeout") {
        initial_perception_timeout_ = parameter.as_double();
      } else if (name == "wait_charge_timeout") {
        wait_charge_timeout_ = parameter.as_double();
      } else if (name == "undock_linear_tolerance") {
        undock_linear_tolerance_ = parameter.as_double();
      } else if (name == "undock_angular_tolerance") {
        undock_angular_tolerance_ = parameter.as_double();
      }
    } else if (type == ParameterType::PARAMETER_STRING) {
      if (name == "base_frame") {
        base_frame_ = parameter.as_string();
      } else if (name == "fixed_frame") {
        fixed_frame_ = parameter.as_string();
      }
    } else if (type == ParameterType::PARAMETER_INTEGER) {
      if (name == "max_retries") {
        max_retries_ = parameter.as_int();
      }
    }
  }

  result.successful = true;
  return result;
}

}  // namespace opennav_docking

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(opennav_docking::DockingServer)
