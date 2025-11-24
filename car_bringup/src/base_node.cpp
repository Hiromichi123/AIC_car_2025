#include <rclcpp/rclcpp.hpp>
#include "base.h"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotBase>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
