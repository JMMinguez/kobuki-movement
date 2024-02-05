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
  publisher_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  subscriber_ = create_subscription<kobuki_ros_interfaces::msg::BumperEvent>(
        "bumper", 10, 
        std::bind(&ForwardNode::bumper_callback, this, _1));
  timer_ = create_wall_timer(
    100ms, std::bind(&ForwardNode::move_forward,this));
}

void
ForwardNode::bumper_callback(const kobuki_ros_interfaces::msg::BumperEvent::SharedPtr msg)
{
    pressed_ = msg->state == kobuki_ros_interfaces::msg::BumperEvent::PRESSED;
}

void
ForwardNode::move_forward()
{
  geometry_msgs::msg::Twist cmd;
  
  if (!start_time_initialized_) {
    start_time_ = std::chrono::steady_clock::now();
    start_time_initialized_ = true;
  }

  auto elapsed_time = std::chrono::steady_clock::now() - start_time_;

  if (elapsed_time > 5000ms || pressed_) {
    cmd.linear.x = 0.0;
    publisher_->publish(cmd);
  } else {
    cmd.linear.x = 0.2;
    publisher_->publish(cmd);
  }
  /* while ((std::chrono::steady_clock::now() - start_time) > 5s)
  {
      cmd.linear.x = 0.2;
      if (pressed_)
      {
          break;
      }
      publisher_->publish(cmd);
  }
  cmd.linear.x = 0.0;
  publisher_->publish(cmd);*/
  
}

}  //  namespace node_forward
