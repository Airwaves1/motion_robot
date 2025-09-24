#include <rclcpp/rclcpp.hpp>
#include "motion_robot/motion_robot.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<motion_robot::MotionRobot>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}




