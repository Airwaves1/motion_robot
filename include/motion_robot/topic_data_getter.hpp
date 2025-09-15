#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <map>
#include <vector>
#include <memory>
#include <functional>
#include <chrono>

namespace motion_robot {

/**
 * @brief VRPN话题数据获取器
 * 
 * 用于订阅VRPN动补数据话题，获取传感器数据并进行关节角度映射
 */
class TopicDataGetter {
public:
    /**
     * @brief 构造函数
     * @param node ROS2节点指针
     */
    TopicDataGetter(rclcpp::Node* node);
    
    /**
     * @brief 析构函数
     */
    ~TopicDataGetter() = default;
    
    /**
     * @brief 初始化数据获取器
     * @param vrpn_tracker_name VRPN跟踪器名称
     * @param sensor_id_offset 传感器ID偏移量
     * @param num_sensors 传感器数量
     * @return 是否初始化成功
     */
    bool initialize(const std::string& vrpn_tracker_name = "MCServer",
                   int sensor_id_offset = 302,
                   int num_sensors = 29);
    
    /**
     * @brief 开始数据获取
     */
    void startDataCollection();
    
    /**
     * @brief 停止数据获取
     */
    void stopDataCollection();
    
    // 关节角度映射功能已移除 - 在G1Controller中处理
    
    /**
     * @brief 获取指定传感器的原始pose数据
     * @param sensor_id 传感器ID
     * @return pose数据指针，如果传感器不存在则返回nullptr
     */
    geometry_msgs::msg::PoseStamped::SharedPtr getPoseData(int sensor_id);
    
    /**
     * @brief 获取传感器统计信息
     * @param sensor_id 传感器ID
     * @return 统计信息结构体
     */
    struct SensorStats {
        int message_count = 0;
        rclcpp::Time last_message_time;
        rclcpp::Time first_message_time;
        bool has_received = false;
        double message_rate = 0.0;
        double last_position[3] = {0.0, 0.0, 0.0};
        double last_orientation[4] = {0.0, 0.0, 0.0, 1.0};
    };
    SensorStats getSensorStats(int sensor_id);
    
    /**
     * @brief 设置数据回调函数
     * @param callback 当接收到新数据时调用的回调函数
     */
    void setDataCallback(std::function<void(int, float)> callback);
    
    // 根位置回调函数已移除
    
    /**
     * @brief 检查是否正在收集数据
     * @return 是否正在收集数据
     */
    bool isCollecting() const { return collecting_; }
    
    /**
     * @brief 获取已接收数据的传感器数量
     * @return 已接收数据的传感器数量
     */
    int getReceivedSensorCount() const;
    
    /**
     * @brief 获取指定传感器的最新pose数据
     * @param sensor_id 传感器ID
     * @return 最新的pose数据，如果没有则返回nullptr
     */
    geometry_msgs::msg::PoseStamped::SharedPtr getLatestPose(int sensor_id) const;
    
    /**
     * @brief 获取测试开始时间
     * @return 测试开始时间
     */
    rclcpp::Time getTestStartTime() const { return test_start_time_; }
    
    /**
     * @brief 设置测试开始时间
     * @param start_time 测试开始时间
     */
    void setTestStartTime(const rclcpp::Time& start_time) { test_start_time_ = start_time; }
    
    // 四元数转欧拉角功能已移除 - 在G1Controller中处理

private:
    // 配置参数
    std::string vrpn_tracker_name_;
    int sensor_id_offset_;
    int num_sensors_;
    bool collecting_;
    
    // ROS2接口
    rclcpp::Node* node_;
    std::vector<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> vrpn_subs_;
    
    // 数据存储
    std::map<int, geometry_msgs::msg::PoseStamped::SharedPtr> latest_poses_;
    std::map<int, SensorStats> sensor_stats_;
    
    
    // 数据回调函数
    std::function<void(int, float)> data_callback_;
    
    // 测试相关
    rclcpp::Time test_start_time_;
    
    // 内部方法
    void createVrpnSubscriptions();
    void onVrpnPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg, int sensor_id);
    void updateSensorStats(int sensor_id, const rclcpp::Time& timestamp);
};

} // namespace motion_robot
