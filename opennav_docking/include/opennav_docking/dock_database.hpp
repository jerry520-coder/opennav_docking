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

#ifndef OPENNAV_DOCKING__DOCK_DATABASE_HPP_
#define OPENNAV_DOCKING__DOCK_DATABASE_HPP_

#include <vector>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"

#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/simple_action_server.hpp"
#include "opennav_docking/utils.hpp"
#include "opennav_docking/types.hpp"
#include "opennav_docking_msgs/srv/reload_database.hpp"

namespace opennav_docking
{
/**
 * @class opennav_docking::DockDatabase
 * @brief An object to contain docks and docking plugins
 */
class DockDatabase
{
public:
  /**
   * @brief A constructor for opennav_docking::DockDatabase
   */
  DockDatabase();

  /**
   * @brief A setup function to populate database
   * @param parent Weakptr to the node to use to get interances and parameters
   * @param tf TF buffer
   * @return If successful
   */

  /**
 * @brief 一个用于填充数据库的设置函数
 * @param parent 指向节点的弱指针，用于获取实例和参数
 * @param tf TF缓冲区
 * @return 是否成功
 */
  bool initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::shared_ptr<tf2_ros::Buffer> tf);

  /**
   * @brief A destructor for opennav_docking::DockDatabase
   */
  ~DockDatabase();

  /**
   * @brief An activation method
   */
  void activate();

  /**
   * @brief An deactivation method
   */
  void deactivate();

  /**
   * @brief Find a dock instance & plugin in the databases from ID
   * @param dock_id Id of dock to find
   * @return Dock pointer
   */

  /**
 * @brief 从ID中查找数据库中的对接实例和插件
 * @param dock_id 要查找的对接ID
 * @return Dock 指针
 */
  Dock * findDock(const std::string & dock_id);

  /**
   * @brief Find a dock plugin to use for a given type
   * @param type Dock type to find plugin for
   * @return Dock plugin pointer
   */

  /**
 * @brief 查找用于给定类型的对接插件
 * @param type 要查找插件的对接类型
 * @return 对接插件指针
 */
  ChargingDock::Ptr findDockPlugin(const std::string & type);

  /**
   * @brief Get the number of docks in the database
   * @return unsigned int Number of dock instances in the database
   */

  /**
 * @brief 获取数据库中的对接数量
 * @return unsigned int 数据库中的对接实例数量
 */
  unsigned int instance_size() const;

  /**
   * @brief Get the number of dock types in the database
   * @return unsigned int Number of dock types in the database
   */

  /**
 * @brief 获取数据库中的对接类型数量
 * @return unsigned int 数据库中的对接类型数量
 */
  unsigned int plugin_size() const;

protected:
  /**
   * @brief Populate database of dock type plugins
   * @param Node Node to get values from
   * @param tf TF buffer
   * @return bool If successful
   */

  /**
 * @brief 填充对接类型插件的数据库
 * @param Node 用于获取值的节点
 * @param tf TF缓冲区
 * @return bool 是否成功
 */
  bool getDockPlugins(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr & node, std::shared_ptr<tf2_ros::Buffer> tf);

  /**
   * @brief Populate database of dock instances
   * @param Node Node to get values from
   */
  bool getDockInstances(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node);

  /**
   * @brief Find a dock instance in the database from ID
   * @param dock_id Id of dock to find
   * @return Dock pointer
   */
  Dock * findDockInstance(const std::string & dock_id);

  /**
   * @brief Service request to reload database of docks
   * @param request Service request
   * @param response Service response
   */

  /**
 * @brief 服务请求，重新加载对接数据库
 * @param request 服务请求
 * @param response 服务响应
 */
  void reloadDbCb(
    const std::shared_ptr<opennav_docking_msgs::srv::ReloadDatabase::Request> request,
    std::shared_ptr<opennav_docking_msgs::srv::ReloadDatabase::Response> response);

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  DockPluginMap dock_plugins_;
  DockMap dock_instances_;
  pluginlib::ClassLoader<opennav_docking_core::ChargingDock> dock_loader_;
  rclcpp::Service<opennav_docking_msgs::srv::ReloadDatabase>::SharedPtr reload_db_service_;
};

}  // namespace opennav_docking

#endif  // OPENNAV_DOCKING__DOCK_DATABASE_HPP_
