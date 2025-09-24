#include "motion_robot/g1_controller.hpp"
#include "motion_robot/motor_crc.hpp"
#include <algorithm>
#include <cmath>

namespace motion_robot {

G1Controller::G1Controller(rclcpp::Node* node) 
    : node_(node), control_frequency_(500.0), state_received_(false), control_mode_(0x0A),
      default_kp_(20.0f), default_kd_(0.5f), enable_arm_swing_simulation_(false), 
      arm_swing_time_(0.0), arm_swing_frequency_(1.0), arm_swing_amplitude_(0.5) {
    
    // 初始化电机使能状态
    for (int i = 0; i < G1_NUM_MOTOR; i++) {
        motor_enabled_[i] = false;
    }
    
}

bool G1Controller::initialize() {
    try {
        // 创建低级命令发布器
        low_cmd_pub_ = node_->create_publisher<unitree_hg::msg::LowCmd>("/lowcmd", 10);
        
        // 订阅低级状态
        low_state_sub_ = node_->create_subscription<unitree_hg::msg::LowState>(
            "/lowstate", 10,
            [this](const unitree_hg::msg::LowState::SharedPtr msg) {
                this->onLowState(msg);
            }
        );
        
        // 创建控制定时器
        control_timer_ = node_->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / control_frequency_)),
            [this]() { this->controlTimerCallback(); }
        );
        
        // 初始化电机命令
        initMotorCommands();
        
        return true;
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "G1Controller初始化失败: %s", e.what());
        return false;
    }
}

void G1Controller::initMotorCommands() {
    // 初始化所有电机命令 - 默认禁用所有电机
    for (int i = 0; i < G1_NUM_MOTOR; i++) {
        low_cmd_.motor_cmd[i].mode = 0;  // 默认禁用电机，等待动补数据启用
        low_cmd_.motor_cmd[i].q = 0.0f;
        low_cmd_.motor_cmd[i].dq = 0.0f;
        low_cmd_.motor_cmd[i].kp = default_kp_;
        low_cmd_.motor_cmd[i].kd = default_kd_;
        low_cmd_.motor_cmd[i].tau = 0.0f;
    }
    
    // 设置控制模式
    low_cmd_.mode_pr = 1; 
    low_cmd_.mode_machine = 1;  // 29DOF模式
    
    // 设置安全的站立姿态 - 所有关节为0度
    setSafeStandingPose();
}

void G1Controller::onLowState(const unitree_hg::msg::LowState::SharedPtr msg) {
    low_state_ = *msg;
    state_received_ = true;
}

void G1Controller::controlTimerCallback() {
    if (state_received_) {
        // 执行手臂摇摆模拟（测试用，已关闭）
        // if (enable_arm_swing_simulation_) {
        //     executeArmSwingSimulation();
        // }
        
        sendCommand();
    }
}

bool G1Controller::setJointPosition(int joint_id, float position) {
    if (!validateJointId(joint_id)) {
        return false;
    }
    
    // 限位检查
    if (!isJointInLimit(joint_id, position)) {
        clampJointPosition(joint_id, position);
    }
    
    low_cmd_.motor_cmd[joint_id].q = position;
    return true;
}

bool G1Controller::setJointVelocity(int joint_id, float velocity) {
    if (!validateJointId(joint_id)) {
        return false;
    }
    
    low_cmd_.motor_cmd[joint_id].dq = velocity;
    return true;
}

bool G1Controller::setJointTorque(int joint_id, float torque) {
    if (!validateJointId(joint_id)) {
        return false;
    }
    
    low_cmd_.motor_cmd[joint_id].tau = torque;
    return true;
}

bool G1Controller::setAllJointPositions(const std::vector<float>& positions) {
    if (positions.size() != G1_NUM_MOTOR) {
        RCLCPP_ERROR(node_->get_logger(), "位置数组大小不匹配: 期望 %d, 实际 %zu", 
                    G1_NUM_MOTOR, positions.size());
        return false;
    }
    
    for (int i = 0; i < G1_NUM_MOTOR; i++) {
        if (!setJointPosition(i, positions[i])) {
            return false;
        }
    }
    return true;
}

bool G1Controller::setAllJointVelocities(const std::vector<float>& velocities) {
    if (velocities.size() != G1_NUM_MOTOR) {
        RCLCPP_ERROR(node_->get_logger(), "速度数组大小不匹配: 期望 %d, 实际 %zu", 
                    G1_NUM_MOTOR, velocities.size());
        return false;
    }
    
    for (int i = 0; i < G1_NUM_MOTOR; i++) {
        if (!setJointVelocity(i, velocities[i])) {
            return false;
        }
    }
    return true;
}

bool G1Controller::setAllJointTorques(const std::vector<float>& torques) {
    if (torques.size() != G1_NUM_MOTOR) {
        RCLCPP_ERROR(node_->get_logger(), "力矩数组大小不匹配: 期望 %d, 实际 %zu", 
                    G1_NUM_MOTOR, torques.size());
        return false;
    }
    
    for (int i = 0; i < G1_NUM_MOTOR; i++) {
        if (!setJointTorque(i, torques[i])) {
            return false;
        }
    }
    return true;
}

bool G1Controller::enableMotor(int joint_id, bool enable) {
    if (joint_id == -1) {
        // 启用/禁用所有电机
        for (int i = 0; i < G1_NUM_MOTOR; i++) {
            motor_enabled_[i] = enable;
            low_cmd_.motor_cmd[i].mode = enable ? control_mode_ : 0;
        }
    } else {
        if (!validateJointId(joint_id)) {
            return false;
        }
        motor_enabled_[joint_id] = enable;
        low_cmd_.motor_cmd[joint_id].mode = enable ? control_mode_ : 0;
    }
    return true;
}

bool G1Controller::setControlMode(uint8_t mode) {
    if (mode != 0x0A && mode != 0x0B && mode != 0x0C) {
        RCLCPP_ERROR(node_->get_logger(), "无效的控制模式: 0x%02X", mode);
        return false;
    }
    
    control_mode_ = mode;
    
    // 更新所有启用的电机
    for (int i = 0; i < G1_NUM_MOTOR; i++) {
        if (motor_enabled_[i]) {
            low_cmd_.motor_cmd[i].mode = control_mode_;
        }
    }
    
    return true;
}

bool G1Controller::setPIDParams(int joint_id, float kp, float kd) {
    if (joint_id == -1) {
        // 设置所有关节的PID参数
        for (int i = 0; i < G1_NUM_MOTOR; i++) {
            low_cmd_.motor_cmd[i].kp = kp;
            low_cmd_.motor_cmd[i].kd = kd;
        }
        default_kp_ = kp;
        default_kd_ = kd;
    } else {
        if (!validateJointId(joint_id)) {
            return false;
        }
        low_cmd_.motor_cmd[joint_id].kp = kp;
        low_cmd_.motor_cmd[joint_id].kd = kd;
    }
    return true;
}

bool G1Controller::sendCommand() {
    if (!low_cmd_pub_) {
        return false;
    }
    
    // 计算CRC校验
    get_crc(low_cmd_);
    
    low_cmd_pub_->publish(low_cmd_);
    return true;
}

float G1Controller::getJointPosition(int joint_id) const {
    if (!validateJointId(joint_id) || !state_received_) {
        return 0.0f;
    }
    return low_state_.motor_state[joint_id].q;
}

float G1Controller::getJointVelocity(int joint_id) const {
    if (!validateJointId(joint_id) || !state_received_) {
        return 0.0f;
    }
    return low_state_.motor_state[joint_id].dq;
}

float G1Controller::getJointTorque(int joint_id) const {
    if (!validateJointId(joint_id) || !state_received_) {
        return 0.0f;
    }
    return low_state_.motor_state[joint_id].tau_est;
}

std::vector<float> G1Controller::getAllJointPositions() const {
    std::vector<float> positions(G1_NUM_MOTOR, 0.0f);
    if (state_received_) {
        for (int i = 0; i < G1_NUM_MOTOR; i++) {
            positions[i] = low_state_.motor_state[i].q;
        }
    }
    return positions;
}

std::vector<float> G1Controller::getAllJointVelocities() const {
    std::vector<float> velocities(G1_NUM_MOTOR, 0.0f);
    if (state_received_) {
        for (int i = 0; i < G1_NUM_MOTOR; i++) {
            velocities[i] = low_state_.motor_state[i].dq;
        }
    }
    return velocities;
}

std::vector<float> G1Controller::getAllJointTorques() const {
    std::vector<float> torques(G1_NUM_MOTOR, 0.0f);
    if (state_received_) {
        for (int i = 0; i < G1_NUM_MOTOR; i++) {
            torques[i] = low_state_.motor_state[i].tau_est;
        }
    }
    return torques;
}

bool G1Controller::isJointInLimit(int joint_id, float position) const {
    if (!validateJointId(joint_id)) {
        return false;
    }
    
    const auto& limits = G1_JOINT_LIMIT[joint_id];
    return position >= limits[0] && position <= limits[1];
}

void G1Controller::setControlFrequency(double frequency) {
    if (frequency <= 0 || frequency > 1000) {
        return;
    }
    
    control_frequency_ = frequency;
    
    if (control_timer_) {
        control_timer_->cancel();
        control_timer_ = node_->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / control_frequency_)),
            [this]() { this->controlTimerCallback(); }
        );
    }
    
}

bool G1Controller::validateJointId(int joint_id) const {
    return joint_id >= 0 && joint_id < G1_NUM_MOTOR;
}

void G1Controller::clampJointPosition(int joint_id, float& position) const {
    if (!validateJointId(joint_id)) {
        return;
    }
    
    const auto& limits = G1_JOINT_LIMIT[joint_id];
    position = std::clamp(position, limits[0], limits[1]);
}

void G1Controller::setSafeStandingPose() {
    // 设置安全的站立姿态 - 所有关节为0度
    for (int i = 0; i < G1_NUM_MOTOR; i++) {
        low_cmd_.motor_cmd[i].mode = 1;  // 启用电机
        low_cmd_.motor_cmd[i].q = 0.0f;  // 目标角度为0
        low_cmd_.motor_cmd[i].dq = 0.0f; // 目标速度为0
        low_cmd_.motor_cmd[i].tau = 0.0f; // 前馈力矩为0
        low_cmd_.motor_cmd[i].kp = (i < 13) ? 100.0f : 50.0f; // 根据关节类型设置增益
        low_cmd_.motor_cmd[i].kd = 1.0f;
    }
    
}


bool G1Controller::processQuaternionData(int joint_id, double quaternion_x, double quaternion_y, 
                                         double quaternion_z, double quaternion_w) {
    if (joint_id < 0 || joint_id >= G1_NUM_MOTOR) {
        return false;
    }

    // 直接使用传入四元数的第一个分量作为已转换的关节角度
    float angle = static_cast<float>(quaternion_x);

    // 若角度过小，可直接忽略（避免噪声）
    if (std::abs(angle) < 0.0001f) {
        return false;
    }

    // 限幅
    if (!isJointInLimit(joint_id, angle)) {
        clampJointPosition(joint_id, angle);
    }

    // 下发电机命令
    low_cmd_.motor_cmd[joint_id].q = angle;
    low_cmd_.motor_cmd[joint_id].mode = 1;  // 启用电机
    low_cmd_.motor_cmd[joint_id].kp = (joint_id < 13) ? 100.0f : 50.0f;
    low_cmd_.motor_cmd[joint_id].kd = 1.0f;

    return true;
}

bool G1Controller::simulateVrpnPoseData(int sensor_id, double quaternion_x, double quaternion_y, 
                                        double quaternion_z, double quaternion_w) {
    // 兼容接口，直接调用统一接口
    return processQuaternionData(sensor_id, quaternion_x, quaternion_y, quaternion_z, quaternion_w);
}

void G1Controller::enableArmSwingSimulation(bool enable) {
    enable_arm_swing_simulation_ = enable;
    arm_swing_time_ = 0.0;
    
    if (enable) {
    } else {
    }
}


void G1Controller::executeArmSwingSimulation() {
    // 更新时间
    arm_swing_time_ += 1.0 / control_frequency_;
    
    // 生成模拟的四元数数据（手臂摇摆动作）
    double t = arm_swing_time_;
    
    // 左臂摇摆
    double left_swing = arm_swing_amplitude_ * std::sin(2.0 * M_PI * arm_swing_frequency_ * t);
    double left_roll = left_swing * 0.1;   // 横滚 - 减小系数
    double left_pitch = left_swing * 0.2;  // 俯仰 - 减小系数
    double left_yaw = left_swing * 0.05;   // 偏航 - 减小系数
    
    // 右臂摇摆（相位相反）
    double right_swing = arm_swing_amplitude_ * std::sin(2.0 * M_PI * arm_swing_frequency_ * t + M_PI);
    double right_roll = right_swing * 0.1;
    double right_pitch = right_swing * 0.2;
    double right_yaw = right_swing * 0.05;
    
    // 将欧拉角转换为四元数
    auto eulerToQuaternion = [](double roll, double pitch, double yaw) -> std::tuple<double, double, double, double> {
        double cr = std::cos(roll * 0.5);
        double sr = std::sin(roll * 0.5);
        double cp = std::cos(pitch * 0.5);
        double sp = std::sin(pitch * 0.5);
        double cy = std::cos(yaw * 0.5);
        double sy = std::sin(yaw * 0.5);
        
        double qw = cr * cp * cy + sr * sp * sy;
        double qx = sr * cp * cy - cr * sp * sy;
        double qy = cr * sp * cy + sr * cp * sy;
        double qz = cr * cp * sy - sr * sp * cy;
        
        return std::make_tuple(qx, qy, qz, qw);
    };
    
    // 统一通过processQuaternionData接口处理所有数据
    // 模拟左臂传感器数据（关节15-21对应左臂）
    auto [left_qx, left_qy, left_qz, left_qw] = eulerToQuaternion(left_roll, left_pitch, left_yaw);
    processQuaternionData(15, left_qx, left_qy, left_qz, left_qw);  // 左肩俯仰
    
    // 模拟右臂传感器数据（关节22-28对应右臂）
    auto [right_qx, right_qy, right_qz, right_qw] = eulerToQuaternion(right_roll, right_pitch, right_yaw);
    processQuaternionData(22, right_qx, right_qy, right_qz, right_qw);  // 右肩俯仰
    
    // 其他手臂关节也进行相应的摇摆
    for (int i = 16; i <= 21; i++) {  // 左臂其他关节
        double joint_swing = left_swing * (0.5 - (i - 15) * 0.05);  // 递减幅度，减小系数
        auto [qx, qy, qz, qw] = eulerToQuaternion(joint_swing * 0.05, joint_swing * 0.1, joint_swing * 0.02);
        processQuaternionData(i, qx, qy, qz, qw);
    }
    
    for (int i = 23; i <= 28; i++) {  // 右臂其他关节
        double joint_swing = right_swing * (0.5 - (i - 22) * 0.05);  // 递减幅度，减小系数
        auto [qx, qy, qz, qw] = eulerToQuaternion(joint_swing * 0.05, joint_swing * 0.1, joint_swing * 0.02);
        processQuaternionData(i, qx, qy, qz, qw);
    }
}

void G1Controller::quaternionToEulerAngles(double qx, double qy, double qz, double qw, 
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
    
    // 使用ZYX顺序 (Yaw-Pitch-Roll) 转换，这是ROS2和机器人学中常用的顺序
    // 根据文档：动补软件中 X向右，Y向前，Z向上
    // Unitree机器人：X前，Y左，Z上
    
    // 首先计算标准ZYX欧拉角
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
    // 动补：X右，Y前，Z上
    // Unitree：X前，Y左，Z上
    // 转换矩阵：[X_unitree] = [0 1 0] [X_mocap]
    //           [Y_unitree]   [-1 0 0] [Y_mocap]  
    //           [Z_unitree]   [0 0 1]  [Z_mocap]
    
    // 将欧拉角从动补坐标系转换到Unitree坐标系
    // 这相当于绕Z轴旋转-90度，然后交换X和Y轴
    // 移除未使用变量
    
    // 应用坐标系转换
    double temp_roll = raw_roll;
    double temp_pitch = raw_pitch;
    double temp_yaw = raw_yaw;
    
    // 转换后的角度
    roll = temp_pitch;   // 动补的pitch -> Unitree的roll
    pitch = -temp_roll;  // 动补的roll -> Unitree的pitch (取反)
    yaw = temp_yaw;      // 动补的yaw -> Unitree的yaw
    
    // 角度范围限制到 [-π, π]
    roll = std::fmod(roll + M_PI, 2*M_PI) - M_PI;
    pitch = std::fmod(pitch + M_PI, 2*M_PI) - M_PI;
    yaw = std::fmod(yaw + M_PI, 2*M_PI) - M_PI;
}

void G1Controller::mapSensorToJointAngles(int sensor_id, double roll, double pitch, double yaw, 
                                          std::map<int, float>& joint_angles) {
    // VRPN传感器到G1关节的映射
    // 根据您的文档，传感器302-330对应关节0-28
    
    float scale_factor = 2;  // 增加缩放因子

    if (sensor_id >= 0 && sensor_id < G1_NUM_MOTOR) {
        int joint_id = sensor_id;
        
        // 根据关节类型进行不同的映射 - 只响应关节能运动的主要轴
        if (joint_id >= 0 && joint_id <= 11) {  // 腿部关节 (0-11)
            switch (joint_id) {
                case 0:   // 左髋俯仰 - 只响应pitch
                    joint_angles[joint_id] = static_cast<float>(pitch * 0.5 * scale_factor);
                    break;
                case 1:   // 左髋横滚 - 只响应roll
                    joint_angles[joint_id] = static_cast<float>(roll * 0.5 * scale_factor);
                    break;
                case 2:   // 左髋偏航 - 只响应yaw
                    joint_angles[joint_id] = static_cast<float>(yaw * 0.5 * scale_factor);
                    break;
                case 3:   // 左膝 - 只响应pitch
                    joint_angles[joint_id] = static_cast<float>(pitch * 0.5 * scale_factor);
                    break;
                case 4:   // 左踝俯仰 - 只响应pitch
                    joint_angles[joint_id] = static_cast<float>(pitch * 0.5 * scale_factor);
                    break;
                case 5:   // 左踝横滚 - 只响应roll
                    joint_angles[joint_id] = static_cast<float>(roll * 0.5 * scale_factor);
                    break;
                case 6:   // 右髋俯仰 - 只响应pitch
                    joint_angles[joint_id] = static_cast<float>(pitch * 0.5 * scale_factor);
                    break;
                case 7:   // 右髋横滚 - 只响应roll
                    joint_angles[joint_id] = static_cast<float>(roll * 0.5 * scale_factor);
                    break;
                case 8:   // 右髋偏航 - 只响应yaw
                    joint_angles[joint_id] = static_cast<float>(yaw * 0.5 * scale_factor);
                    break;
                case 9:   // 右膝 - 只响应pitch
                    joint_angles[joint_id] = static_cast<float>(pitch * 0.5 * scale_factor);
                    break;
                case 10:  // 右踝俯仰 - 只响应pitch
                    joint_angles[joint_id] = static_cast<float>(pitch * 0.5 * scale_factor);
                    break;
                case 11:  // 右踝横滚 - 只响应roll
                    joint_angles[joint_id] = static_cast<float>(roll * 0.5 * scale_factor);
                    break;
            }
        } else if (joint_id >= 12 && joint_id <= 14) {  // 躯干关节 (12-14)
            switch (joint_id) {
                case 12:  // 腰部偏航 - 只响应yaw
                    joint_angles[joint_id] = static_cast<float>(yaw * 0.5 * scale_factor);
                    break;
                case 13:  // 腰部横滚 - 只响应roll
                    joint_angles[joint_id] = static_cast<float>(roll * 0.5 * scale_factor);
                    break;
                case 14:  // 腰部俯仰 - 只响应pitch
                    joint_angles[joint_id] = static_cast<float>(pitch * 0.5 * scale_factor);
                    break;
            }
        } else if (joint_id >= 15 && joint_id <= 21) {  // 左臂关节 (15-21)
            switch (joint_id) {
                case 15:  // 左肩俯仰 - 主要响应pitch
                    joint_angles[joint_id] = static_cast<float>(pitch * 0.5 * scale_factor);
                    break;
                case 16:  // 左肩横滚 - 主要响应roll
                    joint_angles[joint_id] = static_cast<float>(roll * 0.5 * scale_factor);
                    break;
                case 17:  // 左肩偏航 - 主要响应yaw
                    joint_angles[joint_id] = static_cast<float>(yaw * 0.5 * scale_factor);
                    break;
                case 18:  // 左肘 - 主要响应pitch
                    joint_angles[joint_id] = static_cast<float>(pitch * 0.5 * scale_factor);
                    break;
                case 19:  // 左腕横滚 - 主要响应roll
                    joint_angles[joint_id] = static_cast<float>(roll * 0.5 * scale_factor);
                    break;
                case 20:  // 左腕俯仰 - 主要响应pitch
                    joint_angles[joint_id] = static_cast<float>(pitch * 0.5 * scale_factor);
                    break;
                case 21:  // 左腕偏航 - 主要响应yaw
                    joint_angles[joint_id] = static_cast<float>(yaw * 0.5 * scale_factor);
                    break;
            }
        } else if (joint_id >= 22 && joint_id <= 28) {  // 右臂关节 (22-28)
            switch (joint_id) {
                case 22:  // 右肩俯仰 - 主要响应pitch
                    joint_angles[joint_id] = static_cast<float>(pitch * 0.5 * scale_factor);
                    break;
                case 23:  // 右肩横滚 - 主要响应roll
                    joint_angles[joint_id] = static_cast<float>(roll * 0.5 * scale_factor);
                    break;
                case 24:  // 右肩偏航 - 主要响应yaw
                    joint_angles[joint_id] = static_cast<float>(yaw * 0.5 * scale_factor);
                    break;
                case 25:  // 右肘 - 主要响应pitch
                    joint_angles[joint_id] = static_cast<float>(pitch * 0.5 * scale_factor);
                    break;
                case 26:  // 右腕横滚 - 主要响应roll
                    joint_angles[joint_id] = static_cast<float>(roll * 0.5 * scale_factor);
                    break;
                case 27:  // 右腕俯仰 - 主要响应pitch
                    joint_angles[joint_id] = static_cast<float>(pitch * 0.5 * scale_factor);
                    break;
                case 28:  // 右腕偏航 - 主要响应yaw
                    joint_angles[joint_id] = static_cast<float>(yaw * 0.5 * scale_factor);
                    break;
            }
        }
    }
}

} // namespace motion_robot
