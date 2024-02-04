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


#ifdef GO_FORWARD__FUNCTIONS_HPP_
#define GO_FORWARD__FUNCTIONS_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "kobuki_ros_interfaces/msg/bumper_event.hpp"

namespace go_forward
{
class ForwardNode : public rclcpp::Node
{
public:
    ForwardNode();
private:
    void bumper_callback(const kobuki_ros_interfaces::msg::BumperEvent::SharedPtr msg) const;
    void move_forward();
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<kobuki_ros_interfaces::msg::BumperEvent>::SharedPtr subscriber_;
    bool state_;
};
}
#endif