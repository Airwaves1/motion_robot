#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <map>
#include <vector>
#include <chrono>
#include <functional>
#include "motion_robot/joint_mapper.hpp"

namespace motion_robot {

/**
 * @brief VRPN话题数据获取器
 * 
 * 订阅VRPN话题并处理数据，提供回调接口
 */
class TopicDataGetter : public rclcpp::Node {
public:
    /**
     * @brief 构造函数
     * @param node_name 节点名称
     * @param tracker_name VRPN跟踪器名称
     * @param sensor_id_offset 传感器ID偏移量
     * @param num_sensors 传感器数量
     */
    TopicDataGetter(
        const std::string& node_name = "vrpn_data_getter",
        const std::string& tracker_name = "MCServer",
        int sensor_id_offset = 302,
        int num_sensors = 29
    );
    
    /**
     * @brief 析构函数
     */
    ~TopicDataGetter() = default;
    
    /**
     * @brief 设置数据回调函数
     * @param callback 回调函数，参数为(sensor_id, pose, joint_angle)
     */
    void setDataCallback(std::function<void(int, const geometry_msgs::msg::Pose&, float)> callback) {
        data_callback_ = callback;
    }
    
    /**
     * @brief 获取统计信息
     */
    struct Statistics {
        int total_messages = 0;
        int active_sensors = 0;
        std::map<int, int> sensor_message_counts;
        rclcpp::Time start_time;
    };
    Statistics getStatistics() const;

private:
    // 配置参数
    std::string tracker_name_;
    int sensor_id_offset_;
    int num_sensors_;
    
    // 关节映射器
    std::unique_ptr<JointMapper> mapper_;
    
    // ROS2接口
    std::vector<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> vrpn_subs_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    
    // 数据回调
    std::function<void(int, const geometry_msgs::msg::Pose&, float)> data_callback_;
    
    // 统计信息
    mutable Statistics stats_;
    
    // 传感器状态
    struct SensorState {
        bool has_received = false;
        int message_count = 0;
        rclcpp::Time last_message_time;
        geometry_msgs::msg::Pose last_pose;
        float last_joint_angle = 0.0f;
    };
    std::map<int, SensorState> sensor_states_;
    
    // 初始化VRPN订阅
    void initializeVrpnSubscriptions();
    
    // VRPN pose回调函数
    void onVrpnPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg, int sensor_id);
    
    // 验证pose数据有效性
    bool isValidPose(const geometry_msgs::msg::Pose& pose);
};

} // namespace motion_robot
