#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>
#include <map>
#include <vector>
#include <iomanip>
#include <cmath>
#include "motion_robot/topic_data_getter.hpp"
#include "motion_robot/g1_controller.hpp"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace motion_robot {

/**
 * @brief 运动机器人控制类
 * 
 * 用于VRPN动补数据的获取、处理和机器人控制
 */
class MotionRobot : public rclcpp::Node {
public:
    /**
     * @brief 构造函数
     */
    MotionRobot();
    
    /**
     * @brief 析构函数
     */
    ~MotionRobot() = default;

private:
    // 配置参数
    std::string vrpn_tracker_name_;
    int sensor_id_offset_;
    int num_sensors_;
    double print_frequency_;
    
    // ROS2接口
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::TimerBase::SharedPtr status_timer_;
    
    // 数据获取器
    std::shared_ptr<TopicDataGetter> data_getter_;
    
    // G1控制器
    std::shared_ptr<G1Controller> g1_controller_;
    
    // 运行时间跟踪
    rclcpp::Time start_time_;
    
    // 初始化数据获取器
    void initializeDataGetter();
    
    // 初始化G1控制器
    void initializeG1Controller();
    
    // VRPN数据回调
    void onVrpnData(int sensor_id, float joint_angle);
    
    // 打印状态信息
    void printStatus();
};

} // namespace motion_robot
