// Copyright 2021 ROBOTIS CO., LTD.
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

#ifndef VELOCITY_CONTROL_NODE_HPP_
#define VELOCITY_CONTROL_NODE_HPP_

#include <cstdio>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "dynamixel_sdk_custom_interfaces/msg/set_position.hpp"
#include "dynamixel_sdk_custom_interfaces/srv/get_position.hpp"


class VelocityControlNode : public rclcpp::Node
{
public:
  using SetVelocity = dynamixel_sdk_custom_interfaces::msg::SetPosition;  // Reuse SetPosition msg for velocity
  using GetVelocity = dynamixel_sdk_custom_interfaces::srv::GetPosition;  // Reuse GetPosition srv for velocity

  VelocityControlNode();
  virtual ~VelocityControlNode();

private:
  rclcpp::Subscription<SetVelocity>::SharedPtr set_velocity_subscriber_;
  rclcpp::Service<GetVelocity>::SharedPtr get_velocity_server_;

  int present_velocity;
};

#endif  // VELOCITY_CONTROL_NODE_HPP_

