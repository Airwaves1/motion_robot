#pragma once

#include <rclcpp/rclcpp.hpp>
#include <unitree_hg/msg/low_cmd.hpp>
#include <unitree_hg/msg/low_state.hpp>
#include <unitree_hg/msg/motor_cmd.hpp>
#include <map>
#include <vector>
#include <memory>
#include <chrono>
#include "motion_robot/g1_joint.hpp"

namespace motion_robot {

/**
 * @brief G1机器人控制器类
 * 
 * 专门用于29DOF版本的G1人形机器人控制
 * 支持位置控制、速度控制和力矩控制
 */
class G1Controller {
public:
    /**
     * @brief 构造函数
     * @param node ROS2节点指针
     */
    explicit G1Controller(rclcpp::Node* node);
    
    /**
     * @brief 析构函数
     */
    ~G1Controller() = default;
    
    /**
     * @brief 初始化控制器
     * @return 是否初始化成功
     */
    bool initialize();
    
    /**
     * @brief 设置关节目标位置
     * @param joint_id 关节ID (0-28)
     * @param position 目标位置（弧度）
     * @return 是否设置成功
     */
    bool setJointPosition(int joint_id, float position);
    
    /**
     * @brief 设置关节目标速度
     * @param joint_id 关节ID (0-28)
     * @param velocity 目标速度（弧度/秒）
     * @return 是否设置成功
     */
    bool setJointVelocity(int joint_id, float velocity);
    
    /**
     * @brief 设置关节目标力矩
     * @param joint_id 关节ID (0-28)
     * @param torque 目标力矩（Nm）
     * @return 是否设置成功
     */
    bool setJointTorque(int joint_id, float torque);
    
    /**
     * @brief 设置所有关节位置
     * @param positions 29个关节的目标位置（弧度）
     * @return 是否设置成功
     */
    bool setAllJointPositions(const std::vector<float>& positions);
    
    /**
     * @brief 设置所有关节速度
     * @param velocities 29个关节的目标速度（弧度/秒）
     * @return 是否设置成功
     */
    bool setAllJointVelocities(const std::vector<float>& velocities);
    
    /**
     * @brief 设置所有关节力矩
     * @param torques 29个关节的目标力矩（Nm）
     * @return 是否设置成功
     */
    bool setAllJointTorques(const std::vector<float>& torques);
    
    /**
     * @brief 启用/禁用电机
     * @param joint_id 关节ID (0-28)，-1表示所有关节
     * @param enable 是否启用
     * @return 是否设置成功
     */
    bool enableMotor(int joint_id = -1, bool enable = true);
    
    /**
     * @brief 设置控制模式
     * @param mode 控制模式 (0x0A:位置控制, 0x0B:速度控制, 0x0C:力矩控制)
     * @return 是否设置成功
     */
    bool setControlMode(uint8_t mode);
    
    /**
     * @brief 设置PID参数
     * @param joint_id 关节ID (0-28)，-1表示所有关节
     * @param kp 位置增益
     * @param kd 速度增益
     * @return 是否设置成功
     */
    bool setPIDParams(int joint_id, float kp, float kd);
    
    /**
     * @brief 发送控制命令
     * @return 是否发送成功
     */
    bool sendCommand();
    
    /**
     * @brief 处理四元数数据并映射为电机角度（统一接口）
     * @param joint_id 关节ID (0-28)
     * @param quaternion_x 四元数x分量
     * @param quaternion_y 四元数y分量
     * @param quaternion_z 四元数z分量
     * @param quaternion_w 四元数w分量
     * @return 是否处理成功
     */
    bool processQuaternionData(int joint_id, double quaternion_x, double quaternion_y, 
                               double quaternion_z, double quaternion_w);
    
    /**
     * @brief 模拟VRPN pose数据并映射为电机角度（兼容接口）
     * @param sensor_id 传感器ID (0-28)
     * @param quaternion_x 四元数x分量
     * @param quaternion_y 四元数y分量
     * @param quaternion_z 四元数z分量
     * @param quaternion_w 四元数w分量
     * @return 是否处理成功
     */
    bool simulateVrpnPoseData(int sensor_id, double quaternion_x, double quaternion_y, 
                               double quaternion_z, double quaternion_w);
    
    /**
     * @brief 启动手臂摇摆动作模拟
     * @param enable 是否启用模拟
     */
    void enableArmSwingSimulation(bool enable = true);
    
    
    /**
     * @brief 获取关节当前位置
     * @param joint_id 关节ID (0-28)
     * @return 当前位置（弧度），如果无效则返回0
     */
    float getJointPosition(int joint_id) const;
    
    /**
     * @brief 获取关节当前速度
     * @param joint_id 关节ID (0-28)
     * @return 当前速度（弧度/秒），如果无效则返回0
     */
    float getJointVelocity(int joint_id) const;
    
    /**
     * @brief 获取关节当前力矩
     * @param joint_id 关节ID (0-28)
     * @return 当前力矩（Nm），如果无效则返回0
     */
    float getJointTorque(int joint_id) const;
    
    /**
     * @brief 获取所有关节位置
     * @return 29个关节的当前位置
     */
    std::vector<float> getAllJointPositions() const;
    
    /**
     * @brief 获取所有关节速度
     * @return 29个关节的当前速度
     */
    std::vector<float> getAllJointVelocities() const;
    
    /**
     * @brief 获取所有关节力矩
     * @return 29个关节的当前力矩
     */
    std::vector<float> getAllJointTorques() const;
    
    /**
     * @brief 检查关节是否在限位范围内
     * @param joint_id 关节ID (0-28)
     * @param position 位置（弧度）
     * @return 是否在限位范围内
     */
    bool isJointInLimit(int joint_id, float position) const;
    
    /**
     * @brief 获取机器人状态
     * @return 是否已接收到状态数据
     */
    bool isStateReceived() const { return state_received_; }
    
    /**
     * @brief 获取控制频率
     * @return 控制频率（Hz）
     */
    double getControlFrequency() const { return control_frequency_; }
    
    /**
     * @brief 设置控制频率
     * @param frequency 控制频率（Hz）
     */
    void setControlFrequency(double frequency);

private:
    // ROS2接口
    rclcpp::Node* node_;
    rclcpp::Publisher<unitree_hg::msg::LowCmd>::SharedPtr low_cmd_pub_;
    rclcpp::Subscription<unitree_hg::msg::LowState>::SharedPtr low_state_sub_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    
    // 控制命令和状态
    unitree_hg::msg::LowCmd low_cmd_;
    unitree_hg::msg::LowState low_state_;
    
    // 控制参数
    double control_frequency_;
    bool state_received_;
    bool motor_enabled_[G1_NUM_MOTOR];
    uint8_t control_mode_;
    
    // 默认PID参数
    float default_kp_;
    float default_kd_;
    
    
    // VRPN模拟参数
    bool enable_arm_swing_simulation_;
    double arm_swing_time_;
    double arm_swing_frequency_;
    double arm_swing_amplitude_;
    
    // 内部方法
    void onLowState(const unitree_hg::msg::LowState::SharedPtr msg);
    void controlTimerCallback();
    void initMotorCommands();
    void executeArmSwingSimulation();
    void quaternionToEulerAngles(double qx, double qy, double qz, double qw, 
                                double& roll, double& pitch, double& yaw);
    void mapSensorToJointAngles(int sensor_id, double roll, double pitch, double yaw, 
                               std::map<int, float>& joint_angles);
    bool validateJointId(int joint_id) const;
    void clampJointPosition(int joint_id, float& position) const;
    void setSafeStandingPose();
};

} // namespace motion_robot
