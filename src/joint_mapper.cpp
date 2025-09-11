#include "motion_robot/joint_mapper.hpp"
#include <algorithm>
#include <iostream>
#include <Eigen/Dense>

namespace motion_robot {

JointMapper::JointMapper(int sensor_id_offset) : sensor_id_offset_(sensor_id_offset) {
    initializeMappings();
}

void JointMapper::initializeMappings() {
    // 初始化VRPN传感器ID到G1关节ID的映射
    for (int i = 0; i < G1_NUM_MOTOR; i++) {
        sensor_to_joint_map_[sensor_id_offset_ + i] = i;
    }
}

float JointMapper::mapPoseToJointAngle(const geometry_msgs::msg::Pose& pose, int sensor_id) {
    // 获取对应的G1关节ID
    auto it = sensor_to_joint_map_.find(sensor_id);
    if (it == sensor_to_joint_map_.end()) {
        std::cerr << "Warning: Unknown sensor ID " << sensor_id << std::endl;
        return 0.0f;
    }
    
    int joint_id = it->second;
    
    // 提取关节角度
    float angle = extractJointAngle(pose, joint_id);
    
    // 应用关节限制
    return applyJointLimits(joint_id, angle);
}

float JointMapper::applyJointLimits(int joint_id, float angle) {
    // 如果关节限制被禁用，直接返回原角度
    if (!joint_limits_enabled_) {
        return angle;
    }
    
    // 检查关节ID是否有效
    if (joint_id < 0 || joint_id >= G1_NUM_MOTOR) {
        return angle;
    }
    
    // 使用G1_JOINT_LIMIT中的限制
    const auto& limits = G1_JOINT_LIMIT[joint_id];
    return std::clamp(angle, limits[0], limits[1]);
}

JointMapper::EulerAngles JointMapper::quaternionToEuler(const geometry_msgs::msg::Quaternion& quat) {
    EulerAngles euler;
    
    // 四元数转欧拉角
    float sinr_cosp = 2 * (quat.w * quat.x + quat.y * quat.z);
    float cosr_cosp = 1 - 2 * (quat.x * quat.x + quat.y * quat.y);
    euler.roll = std::atan2(sinr_cosp, cosr_cosp);
    
    float sinp = 2 * (quat.w * quat.y - quat.z * quat.x);
    if (std::abs(sinp) >= 1) {
        euler.pitch = std::copysign(M_PI / 2, sinp);
    } else {
        euler.pitch = std::asin(sinp);
    }
    
    float siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y);
    float cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z);
    euler.yaw = std::atan2(siny_cosp, cosy_cosp);
    
    return euler;
}

float JointMapper::extractJointAngle(const geometry_msgs::msg::Pose& pose, int joint_id) {
    // 将四元数转换为旋转矩阵，然后提取特定轴的角度
    Eigen::Quaterniond quat(pose.orientation.w, pose.orientation.x, 
                           pose.orientation.y, pose.orientation.z);
    Eigen::Matrix3d rotation_matrix = quat.toRotationMatrix();
    
    // 根据关节类型提取对应的旋转角度
    switch (joint_id) {
        // 腿部关节 - 基于MuJoCo XML中的关节轴定义
        case LEFT_HIP_PITCH:
        case RIGHT_HIP_PITCH:
            // 髋部俯仰：绕X轴旋转
            return extractRotationAngle(rotation_matrix, 0); // X轴
        case LEFT_HIP_ROLL:
        case RIGHT_HIP_ROLL:
            // 髋部横滚：绕Y轴旋转，左右腿方向相反
            if (joint_id == RIGHT_HIP_ROLL) {
                return -extractRotationAngle(rotation_matrix, 1); // Y轴，取负值
            } else {
                return extractRotationAngle(rotation_matrix, 1); // Y轴
            }
        case LEFT_HIP_YAW:
        case RIGHT_HIP_YAW:
            // 髋部偏航：绕Z轴旋转
            return extractRotationAngle(rotation_matrix, 2); // Z轴
        case LEFT_KNEE:
        case RIGHT_KNEE:
            // 膝盖：绕X轴旋转（俯仰）
            return extractRotationAngle(rotation_matrix, 0); // X轴
        case LEFT_ANKLE_PITCH:
        case RIGHT_ANKLE_PITCH:
            // 踝部俯仰：绕X轴旋转
            return extractRotationAngle(rotation_matrix, 0); // X轴
        case LEFT_ANKLE_ROLL:
        case RIGHT_ANKLE_ROLL:
            // 踝部横滚：绕Y轴旋转
            return extractRotationAngle(rotation_matrix, 1); // Y轴
            
        // 腰部关节
        case WAIST_YAW:
            return extractRotationAngle(rotation_matrix, 2); // Z轴
        case WAIST_ROLL:
            return extractRotationAngle(rotation_matrix, 1); // Y轴
        case WAIST_PITCH:
            return extractRotationAngle(rotation_matrix, 0); // X轴
            
        // 左臂关节
        case LEFT_SHOULDER_PITCH:
            return extractRotationAngle(rotation_matrix, 0); // X轴
        case LEFT_SHOULDER_ROLL:
            return extractRotationAngle(rotation_matrix, 1); // Y轴
        case LEFT_SHOULDER_YAW:
            return extractRotationAngle(rotation_matrix, 2); // Z轴
        case LEFT_ELBOW:
            return extractRotationAngle(rotation_matrix, 0); // X轴
        case LEFT_WRIST_ROLL:
            return extractRotationAngle(rotation_matrix, 0); // X轴
        case LEFT_WRIST_PITCH:
            return extractRotationAngle(rotation_matrix, 1); // Y轴
        case LEFT_WRIST_YAW:
            return extractRotationAngle(rotation_matrix, 2); // Z轴
            
        // 右臂关节
        case RIGHT_SHOULDER_PITCH:
            return extractRotationAngle(rotation_matrix, 0); // X轴
        case RIGHT_SHOULDER_ROLL:
            return -extractRotationAngle(rotation_matrix, 1); // Y轴，取负值
        case RIGHT_SHOULDER_YAW:
            return -extractRotationAngle(rotation_matrix, 2); // Z轴，取负值
        case RIGHT_ELBOW:
            return extractRotationAngle(rotation_matrix, 0); // X轴
        case RIGHT_WRIST_ROLL:
            return extractRotationAngle(rotation_matrix, 0); // X轴
        case RIGHT_WRIST_PITCH:
            return extractRotationAngle(rotation_matrix, 1); // Y轴
        case RIGHT_WRIST_YAW:
            return -extractRotationAngle(rotation_matrix, 2); // Z轴，取负值
            
        default:
            return 0.0f;
    }
}

float JointMapper::extractRotationAngle(const Eigen::Matrix3d& rotation_matrix, int axis) {
    // 从旋转矩阵中提取绕指定轴的旋转角度
    // axis: 0=X轴, 1=Y轴, 2=Z轴
    
    switch (axis) {
        case 0: // X轴 (Roll)
            return std::atan2(rotation_matrix(2, 1), rotation_matrix(2, 2));
        case 1: // Y轴 (Pitch)
            return std::asin(-rotation_matrix(2, 0));
        case 2: // Z轴 (Yaw)
            return std::atan2(rotation_matrix(1, 0), rotation_matrix(0, 0));
        default:
            return 0.0f;
    }
}

} // namespace motion_robot
