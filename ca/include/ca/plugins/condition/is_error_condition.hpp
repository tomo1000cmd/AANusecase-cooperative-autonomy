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

#ifndef LELY_VOA__PLUGINS__CONDITION__IS_ERROR_CONDITION_HPP_
#define LELY_VOA__PLUGINS__CONDITION__IS_ERROR_CONDITION_HPP_

#include <string>

#include "bt_agent/bt_condition_node.hpp"

#include "agent_msgs/srv/standard_action.hpp"


namespace nav2_behavior_tree
{

class IsErrorCondition : public BtConditionNode<agent_msgs::srv::StandardAction>
{
public:
  IsErrorCondition(
    const std::string & service_node_name,
    const std::string & service_name,
    const BT::NodeConfiguration & conf);

  void on_tick() override;
  
  BT::NodeStatus on_success() override;

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        // No input ports
      });
  }
};

}  // namespace nav2_behavior_tree

#endif  // LELY_VOA__PLUGINS__CONDITION__ROUTE_STARTED_CONDITION_HPP_