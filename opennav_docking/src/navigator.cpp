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

#include <chrono>
#include "opennav_docking/navigator.hpp"

namespace opennav_docking
{

using namespace std::chrono_literals;  // NOLINT

Navigator::Navigator(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent)
: node_(parent)
{
  auto node = node_.lock();
  node->declare_parameter("navigator_bt_xml", std::string(""));
  node->get_parameter("navigator_bt_xml", navigator_bt_xml_);
}

void Navigator::activate()
{
  // Need separate callback group and executor to call Nav action within docking action // 需要单独的回调组和执行器来在对接动作中调用导航动作
  auto node = node_.lock();
  callback_group_ = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive,
    false);
  executor_.add_callback_group(callback_group_, node->get_node_base_interface());

   // 创建一个导航到位姿的动作客户端
  nav_to_pose_client_ = rclcpp_action::create_client<Nav2Pose>(
    node->get_node_base_interface(),  // 节点的基础接口
    node->get_node_graph_interface(), // 节点的图接口
    node->get_node_logging_interface(), // 节点的日志接口
    node->get_node_waitables_interface(), // 节点的等待接口
    "navigate_to_pose", callback_group_);
}

void Navigator::deactivate()
{
  nav_to_pose_client_.reset();
}


  /**
 * @brief 一个方法，用于前往特定的位置。
 * 如果导航或通信失败，可能会抛出异常。
 * 阻塞直到完成。
 * @param pose 要前往的位置
 * @param max_staging_duration 到达预设位置的最大时间
 */
void Navigator::goToPose(
  const geometry_msgs::msg::PoseStamped & pose,
  const rclcpp::Duration & max_staging_duration,
  bool recursed)
{
  // 创建导航目标对象
  Nav2Pose::Goal goal;
  goal.pose = pose;                        // 设置目标位置
  goal.behavior_tree = navigator_bt_xml_;  // 设置行为树

  // 将最大等待时间转换为chrono毫秒
  const auto timeout = max_staging_duration.to_chrono<std::chrono::milliseconds>();

  // 等待动作服务器激活
  nav_to_pose_client_->wait_for_action_server(1s);

  // 异步发送导航目标
  auto future_goal_handle = nav_to_pose_client_->async_send_goal(goal);

  // 等待目标处理完成
  if (executor_.spin_until_future_complete(
      future_goal_handle, 2s) == rclcpp::FutureReturnCode::SUCCESS)
  {
    // 获取导航结果
    auto future_result = nav_to_pose_client_->async_get_result(future_goal_handle.get());
    if (executor_.spin_until_future_complete(
        future_result, timeout) == rclcpp::FutureReturnCode::SUCCESS)
    {
      auto result = future_result.get();
      // 检查导航是否成功
      if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        return;  // 导航成功，返回
      }
    }
  }

  // 如果导航失败且未递归调用，则尝试再次调用goToPose方法（单次递归）
  if (!recursed) {
    goToPose(pose, max_staging_duration, true);
    return;
  }

  // 如果导航请求失败且递归调用已尝试，抛出异常
  throw opennav_docking_core::FailedToStage("Navigation request to staging pose failed.");
}

}  // namespace opennav_docking
