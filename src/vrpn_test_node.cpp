#include "motion_robot/motion_robot.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<motion_robot::MotionRobot>();
    
    try {
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("motion_robot"), "异常: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}
