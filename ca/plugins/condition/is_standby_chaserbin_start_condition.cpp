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

#include "ca/plugins/condition/is_standby_chaserbin_start_condition.hpp"

namespace nav2_behavior_tree
{

  IsStandbyChaserbinStartCondition::IsStandbyChaserbinStartCondition(
      const std::string &service_node_name,
      const std::string &service_name,
      const BT::NodeConfiguration &conf)
      : BtConditionNode<agent_msgs::srv::StandardAction>(service_node_name, service_name, conf)
  {
  }

  void IsStandbyChaserbinStartCondition::on_tick()
  {
  }

  BT::NodeStatus IsStandbyChaserbinStartCondition::on_success()
  {
    return response_->success ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

} // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
      [](const std::string &name, const BT::NodeConfiguration &config)
  {
    return std::make_unique<nav2_behavior_tree::IsStandbyChaserbinStartCondition>(
        name, "is_standby_chaser_start", config);
  };

  factory.registerBuilder<nav2_behavior_tree::IsStandbyChaserbinStartCondition>(
      "Is_standby_chaser_start", builder);
}