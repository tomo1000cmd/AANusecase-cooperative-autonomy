// Copyright (c) 2021 Eric Dortmans
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

#include <string>
#include <memory>

#include "voa/plugins/action/get_control_service.hpp"

namespace nav2_behavior_tree
{


GetControlService::GetControlService(
  const std::string & service_node_name,
  const std::string & service_name,
  const BT::NodeConfiguration & conf)
: BtServiceNode<agent_msgs::srv::GetControl>(service_node_name, service_name, conf)
{
}

void GetControlService::on_tick()
{
}

BT::NodeStatus GetControlService::on_success()
{
	setOutput<std::string>("control_type", response_->control_type);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<nav2_behavior_tree::GetControlService>(
        name, "get_control", config);
    };

  factory.registerBuilder<nav2_behavior_tree::GetControlService>(
    "GetControl", builder);
}
