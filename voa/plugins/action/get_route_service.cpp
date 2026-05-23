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

#include "voa/plugins/action/get_route_service.hpp"

namespace nav2_behavior_tree
{


GetRouteService::GetRouteService(
  const std::string & service_node_name,
  const std::string & service_name,
  const BT::NodeConfiguration & conf)
: BtServiceNode<agent_msgs::srv::GetRoute>(service_node_name, service_name, conf)
{
}

void GetRouteService::on_tick()
{
}

BT::NodeStatus GetRouteService::on_success()
{
	setOutput<std::string>("route", response_->route);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<nav2_behavior_tree::GetRouteService>(
        name, "get_route", config);
    };

  factory.registerBuilder<nav2_behavior_tree::GetRouteService>(
    "GetRoute", builder);
}
