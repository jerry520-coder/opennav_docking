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

#include "opennav_docking/dock_database.hpp"

namespace opennav_docking
{

DockDatabase::DockDatabase()
: dock_loader_("opennav_docking_core", "opennav_docking_core::ChargingDock")
{}

DockDatabase::~DockDatabase()
{
  dock_instances_.clear();
  dock_plugins_.clear();
}

bool DockDatabase::initialize(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::shared_ptr<tf2_ros::Buffer> tf)
{
  // 保存父节点的弱指针
  node_ = parent;
  // 锁定弱指针并获取共享指针
  auto node = node_.lock();

  // 尝试获取停靠插件和停靠实例，如果成功则返回true
  if (getDockPlugins(node, tf) && getDockInstances(node)) {
    RCLCPP_INFO(
      node->get_logger(),
      "Docking Server has %u dock types and %u dock instances available.",
      this->plugin_size(), this->instance_size());
    return true;
  }

  // 创建一个服务以便在数据库加载失败时重新加载数据库
  reload_db_service_ = node->create_service<opennav_docking_msgs::srv::ReloadDatabase>(
    "~/reload_database",
    std::bind(
      &DockDatabase::reloadDbCb, this,
      std::placeholders::_1, std::placeholders::_2));

  // 如果初始化失败，则返回false
  return false;
}

void DockDatabase::activate()
{
  DockPluginMap::iterator it;
  for (it = dock_plugins_.begin(); it != dock_plugins_.end(); ++it) {
    it->second->activate();
  }
}

void DockDatabase::deactivate()
{
  DockPluginMap::iterator it;
  for (it = dock_plugins_.begin(); it != dock_plugins_.end(); ++it) {
    it->second->deactivate();
  }
}

void DockDatabase::reloadDbCb(
  const std::shared_ptr<opennav_docking_msgs::srv::ReloadDatabase::Request> request,
  std::shared_ptr<opennav_docking_msgs::srv::ReloadDatabase::Response> response)
{
  DockMap dock_instances;
  if (utils::parseDockFile(request->filepath, node_.lock(), dock_instances)) {
    dock_instances_ = dock_instances;
    response->success = true;
    return;
  }
  response->success = false;
}

Dock * DockDatabase::findDock(const std::string & dock_id)
{
  Dock * dock_instance = findDockInstance(dock_id);
  ChargingDock::Ptr dock_plugin{nullptr};

  if (dock_instance) {
    dock_plugin = findDockPlugin(dock_instance->type);
    if (dock_plugin) {
      // Populate the plugin shared pointer
      dock_instance->plugin = dock_plugin;
      return dock_instance;
    }
    throw opennav_docking_core::DockNotValid("Dock requested has no valid plugin!");
  }
  throw opennav_docking_core::DockNotInDB("Dock ID requested is not in database!");
}

Dock * DockDatabase::findDockInstance(const std::string & dock_id)
{
  auto it = dock_instances_.find(dock_id);
  if (it != dock_instances_.end()) {
    return &(it->second);
  }
  return nullptr;
}

ChargingDock::Ptr DockDatabase::findDockPlugin(const std::string & type)
{
  // If only one dock plugin and type not set, use the default dock //如果只有一个停靠码头插件且未设置类型，则使用默认停靠码头
  if (type.empty() && dock_plugins_.size() == 1) {
    return dock_plugins_.begin()->second;
  }

  auto it = dock_plugins_.find(type);
  if (it != dock_plugins_.end()) {
    return it->second;
  }
  return nullptr;
}

bool DockDatabase::getDockPlugins(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
  std::shared_ptr<tf2_ros::Buffer> tf)
{
  // 声明一个字符串向量来存储停靠插件的名称
  std::vector<std::string> docks_plugins;

  // 检查节点是否有'dock_plugins'参数，如果没有则声明该参数
  if (!node->has_parameter("dock_plugins")) {
    node->declare_parameter("dock_plugins", rclcpp::ParameterType::PARAMETER_STRING_ARRAY);
  }

  // 获取'dock_plugins'参数的值，如果获取失败则记录错误日志并返回false
  if (!node->get_parameter("dock_plugins", docks_plugins)) {
    RCLCPP_ERROR(node->get_logger(), "Charging dock plugins not given!");
    return false;
  }

  // 检查插件列表是否为空，如果为空则记录错误日志并返回false
  if (docks_plugins.size() < 1u) {
    RCLCPP_ERROR(node->get_logger(), "Charging dock plugins empty! Must provide 1.");
    return false;
  }

  // 遍历插件列表并尝试创建每个插件的实例
  for (size_t i = 0; i != docks_plugins.size(); i++) {
    try {
      // 获取插件类型
      std::string plugin_type = nav2_util::get_plugin_type_param(node, docks_plugins[i]);
      // 使用插件加载器创建插件实例
      opennav_docking_core::ChargingDock::Ptr dock = dock_loader_.createUniqueInstance(plugin_type);
      RCLCPP_INFO(
        node->get_logger(), "Created charging dock plugin %s of type %s",
        docks_plugins[i].c_str(), plugin_type.c_str());
      // 配置插件
      dock->configure(node, docks_plugins[i], tf);
      // 将插件添加到插件映射中
      dock_plugins_.insert({docks_plugins[i], dock});
    } catch (const std::exception & ex) {
      // 如果创建插件失败，记录致命错误日志并返回false
      RCLCPP_FATAL(
        node->get_logger(), "Failed to create Charging Dock plugin. Exception: %s",
        ex.what());
      return false;
    }
  }

  // 如果所有插件创建和配置成功，返回true
  return true;
}

bool DockDatabase::getDockInstances(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node)
{
  // 使用别名简化参数类型
  using rclcpp::ParameterType::PARAMETER_STRING; //参数字符串
  using rclcpp::ParameterType::PARAMETER_STRING_ARRAY;//参数字符串数组

  // 尝试从单独的文件中获取停靠数据
  std::string dock_filepath;
  // 如果节点没有'dock_database'参数，则声明该参数
  if (!node->has_parameter("dock_database")) {
    node->declare_parameter("dock_database", PARAMETER_STRING);
  }
  // 获取'dock_database'参数的值
  if (node->get_parameter("dock_database", dock_filepath)) {
    RCLCPP_INFO(
      node->get_logger(), "Loading dock from database file  %s.", dock_filepath.c_str());
    try {
      // 解析停靠文件并将结果存储在dock_instances_中
      return utils::parseDockFile(dock_filepath, node, dock_instances_);
    } catch (YAML::ParserException & e) {
      // 如果解析文件失败，记录错误日志并返回false
      RCLCPP_ERROR(
        node->get_logger(),
        "Dock database (%s) is malformed: %s.", dock_filepath.c_str(), e.what());
      return false;
    }
    return true;
  }

  // 尝试从参数文件中获取停靠数据
  std::vector<std::string> docks_param;
  // 如果节点没有'docks'参数，则声明该参数
  if (!node->has_parameter("docks")) {
    node->declare_parameter("docks", PARAMETER_STRING_ARRAY);
  }
  // 获取'docks'参数的值
  if (node->get_parameter("docks", docks_param)) {
    RCLCPP_INFO(node->get_logger(), "Loading docks from parameter file.");
    // 解析停靠参数并将结果存储在dock_instances_中
    return utils::parseDockParams(docks_param, node, dock_instances_);
  }

  // 如果既没有文件路径也没有参数，记录错误日志并返回false
  RCLCPP_ERROR(
    node->get_logger(),
    "Dock database filepath nor dock parameters set. Unable to perform docking actions.");
  return false;
}

unsigned int DockDatabase::plugin_size() const
{
  return dock_plugins_.size();
}

unsigned int DockDatabase::instance_size() const
{
  return dock_instances_.size();
}

}  // namespace opennav_docking
