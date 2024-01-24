// Copyright (c) 2023 Takumi Asada
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

#ifndef JOINT_PUB_NODE_HPP_
#define JOINT_PUB_NODE_HPP_

/******************************************************************************/
/* include                                                                    */
/******************************************************************************/
#include <rclcpp/rclcpp.hpp>
#include "dynamixel_sdk_custom_interfaces/msg/set_position.hpp"
#include "dynamixel_sdk_custom_interfaces/srv/get_position.hpp"


class JointPubNode : public rclcpp::Node
{
public:
  using SetPosition = dynamixel_sdk_custom_interfaces::msg::SetPosition;

  JointPubNode();

private:
  void publishData();
  rclcpp::Publisher<SetPosition>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_; 
  std::vector<int> positions_1_ = {2024, 2024};
  std::vector<int> velocity_1_ = {50, 50};
  std::vector<int> acceleration_1_ = {5, 5};  
  /*
  std::vector<int> positions_1_ = {0, 1022, 3068};
  std::vector<int> positions_2_ = {0, 3068, 1022};
  std::vector<int> velocity_1_ = {100, 200, 100};
  std::vector<int> velocity_2_ = {100, 50, 200};
  std::vector<int> acceleration_1_ = {10, 30, 5};
  std::vector<int> acceleration_2_ = {10, 5, 30};
  */
  size_t current_position_index_ = 0;
};

#endif  // JOINT_PUB_NODE_HPP_
