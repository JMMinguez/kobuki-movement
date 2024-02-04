#include <iostream>

#include "rclcpp/rclcpp.hpp"

#include "go_forward_functions/functions.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<go_forward::ForwardNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}