#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <map>
#include <vector>
#include <cmath>
#include "motion_robot/g1_joint.hpp"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace motion_robot {

/**
 * @brief VRPN到G1关节角度的映射器
 * 
 * 将VRPN动补数据转换为G1机器人的29个关节角度
 */
class JointMapper {
public:
    /**
     * @brief 构造函数
     * @param sensor_id_offset 传感器ID偏移量
     */
    JointMapper(int sensor_id_offset = 300);
    
    /**
     * @brief 析构函数
     */
    ~JointMapper() = default;
    
    /**
     * @brief 将VRPN pose数据映射到G1关节角度
     * @param pose VRPN pose数据
     * @param sensor_id VRPN传感器ID
     * @return 对应的G1关节角度（弧度）
     */
    float mapPoseToJointAngle(const geometry_msgs::msg::Pose& pose, int sensor_id);
    
    /**
     * @brief 应用G1关节角度限制
     * @param joint_id 关节ID
     * @param angle 原始角度
     * @return 限制后的角度
     */
    float applyJointLimits(int joint_id, float angle);
    
    /**
     * @brief 从四元数提取欧拉角
     * @param quat 四元数
     * @return 欧拉角结构体
     */
    struct EulerAngles {
        float roll, pitch, yaw;
    };
    EulerAngles quaternionToEuler(const geometry_msgs::msg::Quaternion& quat);
    
    /**
     * @brief 设置传感器ID偏移量
     * @param offset 传感器ID偏移量（默认300）
     */
    void setSensorIdOffset(int offset) { sensor_id_offset_ = offset; }
    
    /**
     * @brief 启用/禁用关节限制
     * @param enable 是否启用关节限制
     */
    void setJointLimitsEnabled(bool enable) { joint_limits_enabled_ = enable; }

private:
    // 传感器ID偏移量
    int sensor_id_offset_ = 300;
    
    // 关节限制开关
    bool joint_limits_enabled_ = true;
    
    // VRPN传感器ID到G1关节ID的映射
    std::map<int, int> sensor_to_joint_map_;
    
    // 初始化映射关系
    void initializeMappings();
    
    // 从pose中提取关节角度
    float extractJointAngle(const geometry_msgs::msg::Pose& pose, int joint_id);
    
    // 从旋转矩阵提取指定轴的旋转角度
    float extractRotationAngle(const Eigen::Matrix3d& rotation_matrix, int axis);
};

} // namespace motion_robot
