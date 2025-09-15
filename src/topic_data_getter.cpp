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
    // 创建状态发布器
    status_pub_ = node_->create_publisher<std_msgs::msg::String>("/motion_robot/status", 10);
    
    // 初始化测试开始时间
    test_start_time_ = node_->get_clock()->now();
    
    RCLCPP_DEBUG(node_->get_logger(), "TopicDataGetter已创建");
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
    
    RCLCPP_INFO(node_->get_logger(), "TopicDataGetter初始化完成 - %d个传感器", num_sensors_);
    
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
        
        RCLCPP_DEBUG(node_->get_logger(), "订阅话题: %s", topic.c_str());
    }
    
    RCLCPP_DEBUG(node_->get_logger(), "已创建 %zu 个VRPN话题订阅", vrpn_subs_.size());
}

void TopicDataGetter::onVrpnPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg, int sensor_id) {
    // 更新最新pose数据
    latest_poses_[sensor_id] = msg;
    
    // 存储原始pose数据，关节角度映射在G1Controller中处理
    latest_joint_angles_[sensor_id] = 0.0f;  // 占位符，实际映射在控制器中
    
    // 更新统计信息
    updateSensorStats(sensor_id, msg->header.stamp);
    
    // 调用数据回调函数
    if (data_callback_) {
        data_callback_(sensor_id, 0.0f);  // 传递占位符，实际映射在控制器中
    }
    
    RCLCPP_DEBUG(node_->get_logger(), "传感器 %d: 接收到VRPN数据", sensor_id);
}

void TopicDataGetter::updateSensorStats(int sensor_id, const rclcpp::Time& timestamp) {
    auto& stats = sensor_stats_[sensor_id];
    
    stats.message_count++;
    stats.last_message_time = timestamp;
    
    if (!stats.has_received) {
        stats.first_message_time = timestamp;
        stats.has_received = true;
        RCLCPP_DEBUG(node_->get_logger(), "传感器 %d 首次接收到数据", sensor_id);
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
        RCLCPP_WARN(node_->get_logger(), "数据收集已经在进行中");
        return;
    }
    
    collecting_ = true;
    
    // 创建状态发布定时器 (默认1Hz)
    status_timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(1000),
        [this]() { this->statusTimerCallback(); }
    );
    
    RCLCPP_DEBUG(node_->get_logger(), "开始数据收集");
}

void TopicDataGetter::stopDataCollection() {
    if (!collecting_) {
        RCLCPP_WARN(node_->get_logger(), "数据收集未在进行中");
        return;
    }
    
    collecting_ = false;
    
    if (status_timer_) {
        status_timer_->cancel();
        status_timer_.reset();
    }
    
    RCLCPP_DEBUG(node_->get_logger(), "停止数据收集");
}

float TopicDataGetter::getJointAngle(int sensor_id) {
    auto it = latest_joint_angles_.find(sensor_id);
    if (it != latest_joint_angles_.end()) {
        return it->second;
    }
    return 0.0f;
}

std::map<int, float> TopicDataGetter::getAllJointAngles() {
    return latest_joint_angles_;
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

void TopicDataGetter::statusTimerCallback() {
    if (!collecting_) return;
    
    publishStatus();
}

void TopicDataGetter::publishStatus() {
    auto current_time = node_->get_clock()->now();
    int received_sensors = getReceivedSensorCount();
    int total_messages = 0;
    
    for (const auto& [sensor_id, stats] : sensor_stats_) {
        total_messages += stats.message_count;
    }
    
    // 发布状态消息
    auto status_msg = std_msgs::msg::String();
    status_msg.data = "数据收集状态 - 已接收传感器: " + std::to_string(received_sensors) + 
                     "/" + std::to_string(num_sensors_) + 
                     ", 总消息数: " + std::to_string(total_messages) +
                     ", 收集状态: " + (collecting_ ? "进行中" : "已停止");
    status_pub_->publish(status_msg);
    
    // 只显示简要状态
    RCLCPP_DEBUG(node_->get_logger(), "数据收集状态: %d/%d传感器, %d消息", 
                received_sensors, num_sensors_, total_messages);
}

void TopicDataGetter::quaternionToEuler(double qx, double qy, double qz, double qw, 
                                       double& roll, double& pitch, double& yaw) {
    // 四元数归一化
    double norm = std::sqrt(qx*qx + qy*qy + qz*qz + qw*qw);
    if (norm < 1e-8) {
        roll = pitch = yaw = 0.0;
        return;
    }
    
    qx /= norm;
    qy /= norm;
    qz /= norm;
    qw /= norm;
    
    // 使用ZYX顺序 (Yaw-Pitch-Roll) 转换
    // 动补软件中 X向右，Y向前，Z向上
    // Unitree机器人：X前，Y左，Z上
    
    // 计算标准ZYX欧拉角
    double sinr_cosp = 2 * (qw * qx + qy * qz);
    double cosr_cosp = 1 - 2 * (qx * qx + qy * qy);
    double raw_roll = std::atan2(sinr_cosp, cosr_cosp);
    
    double sinp = 2 * (qw * qy - qz * qx);
    double raw_pitch;
    if (std::abs(sinp) >= 1) {
        raw_pitch = std::copysign(M_PI / 2, sinp);
    } else {
        raw_pitch = std::asin(sinp);
    }
    
    double siny_cosp = 2 * (qw * qz + qx * qy);
    double cosy_cosp = 1 - 2 * (qy * qy + qz * qz);
    double raw_yaw = std::atan2(siny_cosp, cosy_cosp);
    
    // 坐标系转换：从动补坐标系转换到Unitree坐标系
    roll = raw_pitch;    // 动补的pitch -> Unitree的roll
    pitch = -raw_roll;   // 动补的roll -> Unitree的pitch (取反)
    yaw = raw_yaw;       // 动补的yaw -> Unitree的yaw
    
    // 角度范围限制到 [-π, π]
    roll = std::fmod(roll + M_PI, 2*M_PI) - M_PI;
    pitch = std::fmod(pitch + M_PI, 2*M_PI) - M_PI;
    yaw = std::fmod(yaw + M_PI, 2*M_PI) - M_PI;
}

} // namespace motion_robot
