// Copyright 2024 Intelligent Robotics Lab
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

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "kobuki_ros_interfaces/msg/bumper_event.hpp"

#include "node_forward/ForwardNode.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace node_forward
{

ForwardNode::ForwardNode()
: Node("forward_node")
{
  publisher_ = create_publisher<geometry_msgs::msg::Twist>("vel", 10);
  subscriber_ = create_subscription<kobuki_ros_interfaces::msg::BumperEvent>(
        "bumper", 10, 
        std::bind(&ForwardNode::bumper_callback, this, _1));
}

void
ForwardNode::bumper_callback(const kobuki_ros_interfaces::msg::BumperEvent::SharedPtr msg)
{
    pressed_ = msg->state;
}

void
ForwardNode::move_forward()
{
  geometry_msgs::msg::Twist cmd;
  auto start_time = std::chrono::steady_clock::now();

  while ((std::chrono::steady_clock::now() - start_time) < 5s)
  {
      cmd.linear.x = 0.2;
      if (pressed_)
      {
          break;
      }
      publisher_->publish(cmd);
  }
  cmd.linear.x = 0.0;
  publisher_->publish(cmd);
}

}  //  namespace node_forward
