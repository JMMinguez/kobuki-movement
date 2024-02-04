#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "kobuki_ros_interfaces/msg/bumper_event.hpp"

#include "go_forward_functions/functions.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders::_1;

namespace go_forward
{

ForwardNode::ForwardNode()
: Node("forward_node")
{
    publisher_ = create_publisher<geometry_msgs::msg::Twist>("int_topic", 10);
    subscriber_ = create_subscription<kobuki_ros_interfaces::msg::BumperEvent>(
        "int_topic", 10, 
        std::bind(&ForwardNode::bumper_callback, this, _1))
    timer_ = create_wall_timer(
        5s, std::bind(&ForwardNode::timer_callback, this));
        //5s, std::bind(%ForwardNode::timer_callback, this));
}

void ForwardNode::bumper_callback(const kobuki_ros_interfaces::msg::BumperEvent::SharedPtr msg) const
{
    state_ = msg->state;
}


/*
void ForwardNode::move_forward()
{
    geometry_msgs::Twist cmd;

    while(timer_ < 5)
    {
        cmd.linear.x = 0.2;
        if (state_)
        {
            break;
        }
        publisher_.publish(cmd);
    }

    cmd.linear.x = 0;
    publisher_.publish(cmd);

}
}
*/
void ForwardNode::move_forward()
{
    geometry_msgs::msg::Twist cmd;

    auto start_time = now();
    while ((now() - start_time) < 5s)
    {
        cmd.linear.x = 0.2;
        if (state_)
        {
            break;
        }
        publisher_->publish(cmd);
        rclcpp::spin_some(this->get_node_base_interface());
    }

    cmd.linear.x = 0;
    publisher_->publish(cmd);
}

} 