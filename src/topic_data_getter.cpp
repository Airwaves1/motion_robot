#include "motion_robot/topic_data_getter.hpp"
#include <algorithm>
#include <iomanip>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace motion_robot {

TopicDataGetter::TopicDataGetter(rclcpp::Node* node) 
    : collecting_(false), node_(node) {
    test_start_time_ = node_->get_clock()->now();
}

bool TopicDataGetter::initialize(const std::string& vrpn_tracker_name,
                                int sensor_id_offset,
                                int num_sensors) {
    vrpn_tracker_name_ = vrpn_tracker_name;
    sensor_id_offset_ = sensor_id_offset;
    num_sensors_ = num_sensors;
    
    
    // 初始化传感器统计信息
    for (int i = 0; i < num_sensors_; i++) {
        int sensor_id = sensor_id_offset_ + i;
        sensor_stats_[sensor_id] = SensorStats();
    }
    
    // 创建VRPN话题订阅
    createVrpnSubscriptions();
    
    
    return true;
}

void TopicDataGetter::createVrpnSubscriptions() {
    vrpn_subs_.clear();
    
    for (int i = 0; i < num_sensors_; i++) {
        int sensor_id = sensor_id_offset_ + i;
        std::string topic = "/vrpn_mocap/" + vrpn_tracker_name_ + "/pose" + std::to_string(sensor_id);
        
        auto sub = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
            topic, rclcpp::SensorDataQoS(),
            [this, sensor_id](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                this->onVrpnPose(msg, sensor_id);
            }
        );
        vrpn_subs_.push_back(sub);
        
    }
    
}

void TopicDataGetter::onVrpnPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg, int sensor_id) {
    // 更新最新pose数据
    latest_poses_[sensor_id] = msg;
    
    latest_joint_angles_[sensor_id] = 0.0f;
    
    // 更新统计信息
    updateSensorStats(sensor_id, msg->header.stamp);
    
    // 调用数据回调函数
    if (data_callback_) {
        data_callback_(sensor_id, 0.0f);  // 传递占位符，实际映射在控制器中
    }
    
}

void TopicDataGetter::updateSensorStats(int sensor_id, const rclcpp::Time& timestamp) {
    auto& stats = sensor_stats_[sensor_id];
    
    stats.message_count++;
    stats.last_message_time = timestamp;
    
    if (!stats.has_received) {
        stats.first_message_time = timestamp;
        stats.has_received = true;
    }
    
    // 计算消息频率
    if (stats.message_count > 1) {
        double elapsed_time = (timestamp - stats.first_message_time).seconds();
        if (elapsed_time > 0) {
            stats.message_rate = stats.message_count / elapsed_time;
        }
    }
    
    // 更新位置和姿态信息
    auto pose_it = latest_poses_.find(sensor_id);
    if (pose_it != latest_poses_.end()) {
        const auto& pose = pose_it->second->pose;
        stats.last_position[0] = pose.position.x;
        stats.last_position[1] = pose.position.y;
        stats.last_position[2] = pose.position.z;
        stats.last_orientation[0] = pose.orientation.x;
        stats.last_orientation[1] = pose.orientation.y;
        stats.last_orientation[2] = pose.orientation.z;
        stats.last_orientation[3] = pose.orientation.w;
    }
}

void TopicDataGetter::startDataCollection() {
    if (collecting_) {
        return;
    }
    
    collecting_ = true;
    
}

void TopicDataGetter::stopDataCollection() {
    if (!collecting_) {
        return;
    }
    
    collecting_ = false;
    
    // 状态定时器已移除
    
}

geometry_msgs::msg::PoseStamped::SharedPtr TopicDataGetter::getPoseData(int sensor_id) {
    auto it = latest_poses_.find(sensor_id);
    if (it != latest_poses_.end()) {
        return it->second;
    }
    return nullptr;
}

TopicDataGetter::SensorStats TopicDataGetter::getSensorStats(int sensor_id) {
    auto it = sensor_stats_.find(sensor_id);
    if (it != sensor_stats_.end()) {
        return it->second;
    }
    return SensorStats();
}

void TopicDataGetter::setDataCallback(std::function<void(int, float)> callback) {
    data_callback_ = callback;
}

int TopicDataGetter::getReceivedSensorCount() const {
    int count = 0;
    for (const auto& [sensor_id, stats] : sensor_stats_) {
        if (stats.has_received) {
            count++;
        }
    }
    return count;
}

geometry_msgs::msg::PoseStamped::SharedPtr TopicDataGetter::getLatestPose(int sensor_id) const {
    auto it = latest_poses_.find(sensor_id);
    if (it != latest_poses_.end()) {
        return it->second;
    }
    return nullptr;
}



} // namespace motion_robot
