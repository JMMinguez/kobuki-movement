#ifdef GO_FORWARD__FORWARDNODE_HPP_
#define GO_FORWARD__FORWARDNODE_HPP_

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
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<kobuki_ros_interfaces::msg::BumperEvent>::SharedPtr subscriber_;
    bool state_;
};
}
#endif