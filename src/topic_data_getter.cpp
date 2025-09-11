#include "motion_robot/topic_data_getter.hpp"
#include <algorithm>
#include <iostream>

namespace motion_robot {

TopicDataGetter::TopicDataGetter(
    const std::string& node_name,
    const std::string& tracker_name,
    int sensor_id_offset,
    int num_sensors
) : Node(node_name),
    tracker_name_(tracker_name),
    sensor_id_offset_(sensor_id_offset),
    num_sensors_(num_sensors) {
    
    // 初始化关节映射器
    mapper_ = std::make_unique<JointMapper>(sensor_id_offset_);
    
    // 初始化统计信息
    stats_.start_time = this->get_clock()->now();
    
    // 创建状态发布器
    status_pub_ = this->create_publisher<std_msgs::msg::String>("/vrpn_data_status", 10);
    
    // 初始化VRPN订阅
    initializeVrpnSubscriptions();
    
    RCLCPP_INFO(this->get_logger(), "VRPN数据获取器已启动");
    RCLCPP_INFO(this->get_logger(), "跟踪器名称: %s", tracker_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "传感器数量: %d", num_sensors_);
    RCLCPP_INFO(this->get_logger(), "传感器ID范围: %d - %d", sensor_id_offset_, sensor_id_offset_ + num_sensors_ - 1);
}

void TopicDataGetter::initializeVrpnSubscriptions() {
    // 为每个传感器创建VRPN订阅
    for (int i = 0; i < num_sensors_; i++) {
        int sensor_id = sensor_id_offset_ + i;
        std::string topic = "/vrpn_mocap/" + tracker_name_ + "/pose" + std::to_string(sensor_id);
        
        auto sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            topic, rclcpp::SensorDataQoS(),
            [this, sensor_id](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                this->onVrpnPose(msg, sensor_id);
            }
        );
        vrpn_subs_.push_back(sub);
        
        RCLCPP_DEBUG(this->get_logger(), "订阅话题: %s", topic.c_str());
    }
    
    RCLCPP_INFO(this->get_logger(), "已创建 %zu 个VRPN订阅", vrpn_subs_.size());
}

void TopicDataGetter::onVrpnPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg, int sensor_id) {
    auto& state = sensor_states_[sensor_id];
    
    // 更新统计信息
    state.message_count++;
    state.last_message_time = msg->header.stamp;
    stats_.total_messages++;
    
    if (!state.has_received) {
        state.has_received = true;
        stats_.active_sensors++;
        RCLCPP_INFO(this->get_logger(), "传感器 %d 首次接收到数据", sensor_id);
    }
    
    // 保存最新的pose数据
    state.last_pose = msg->pose;
    
    // 数据验证
    if (!isValidPose(msg->pose)) {
        RCLCPP_WARN(this->get_logger(), "传感器 %d 接收到无效的pose数据", sensor_id);
        return;
    }
    
    // 将VRPN数据映射到G1关节角度
    float joint_angle = mapper_->mapPoseToJointAngle(msg->pose, sensor_id);
    state.last_joint_angle = joint_angle;
    
    // 更新传感器消息计数
    stats_.sensor_message_counts[sensor_id] = state.message_count;
    
    // 调用用户回调函数
    if (data_callback_) {
        data_callback_(sensor_id, msg->pose, joint_angle);
    }
    
    // 每100次消息打印一次调试信息
    if (state.message_count % 100 == 0) {
        RCLCPP_DEBUG(this->get_logger(),
            "传感器 %d: 位置(%.2f,%.2f,%.2f) -> 关节角度: %.3f",
            sensor_id, 
            msg->pose.position.x, msg->pose.position.y, msg->pose.position.z,
            joint_angle);
    }
}

bool TopicDataGetter::isValidPose(const geometry_msgs::msg::Pose& pose) {
    // 检查位置数据
    if (std::isnan(pose.position.x) || std::isnan(pose.position.y) || std::isnan(pose.position.z)) {
        return false;
    }
    
    // 检查四元数数据
    if (std::isnan(pose.orientation.x) || std::isnan(pose.orientation.y) || 
        std::isnan(pose.orientation.z) || std::isnan(pose.orientation.w)) {
        return false;
    }
    
    // 检查四元数归一化
    double norm = std::sqrt(pose.orientation.x * pose.orientation.x + 
                           pose.orientation.y * pose.orientation.y + 
                           pose.orientation.z * pose.orientation.z + 
                           pose.orientation.w * pose.orientation.w);
    if (std::abs(norm - 1.0) > 0.1) {
        return false;
    }
    
    return true;
}

TopicDataGetter::Statistics TopicDataGetter::getStatistics() const {
    return stats_;
}

} // namespace motion_robot
