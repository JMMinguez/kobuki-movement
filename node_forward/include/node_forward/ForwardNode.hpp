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

#ifndef NODE_FORWARD__FORWARDNODE_HPP_
#define NODE_FORWARD__FORWARDNODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "kobuki_ros_interfaces/msg/bumper_event.hpp"

namespace node_forward
{

class ForwardNode : public rclcpp::Node
{
public:
  ForwardNode();
  void move_forward();
  void bumper_callback(const kobuki_ros_interfaces::msg::BumperEvent::SharedPtr msg);

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<kobuki_ros_interfaces::msg::BumperEvent>::SharedPtr subscriber_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool pressed_;
  bool start_time_initialized_;
  std::chrono::steady_clock::time_point start_time_;
};

}  //  namespace node_forward

#endif  // NODE_FORWARD__FORWARDNODE_HPP_
