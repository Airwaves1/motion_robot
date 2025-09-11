# 宇树G1机器人电机命令详解

## 概述

宇树G1机器人采用29个电机控制，每个电机通过`unitree_hg::msg::LowCmd`消息进行控制。本文档详细解释电机命令的各个参数含义、控制模式以及实际应用。

## 电机命令结构

### LowCmd消息结构

```cpp
unitree_hg::msg::LowCmd low_cmd;

// 每个电机命令包含以下参数
low_cmd.motor_cmd[i].mode = 1;     // 电机模式
low_cmd.motor_cmd[i].q = 0.0;      // 目标位置（弧度）
low_cmd.motor_cmd[i].dq = 0.0;     // 目标速度（弧度/秒）
low_cmd.motor_cmd[i].tau = 0.0;    // 目标力矩（牛·米）
low_cmd.motor_cmd[i].kp = 100.0;   // 位置增益
low_cmd.motor_cmd[i].kd = 1.0;     // 速度增益
```

## 参数详解

### 1. mode - 电机模式

| 值 | 模式 | 说明 |
|---|---|---|
| 0 | 停止模式 | 电机停止，不响应任何命令 |
| 1 | 位置控制模式 | 电机根据目标位置进行PID控制 |
| 2 | 速度控制模式 | 电机根据目标速度进行控制 |
| 3 | 力矩控制模式 | 电机根据目标力矩进行控制 |

**推荐使用**：位置控制模式（mode = 1），这是最常用且最稳定的控制方式。

### 2. q - 目标位置

- **单位**：弧度（rad）
- **范围**：根据关节类型而定
- **作用**：设置电机期望到达的角度位置

**关节角度范围**：
```cpp
// 腿部关节
left_hip_pitch_joint:  [-2.5307, 2.8798] rad
left_hip_roll_joint:   [-0.5236, 2.9671] rad
left_hip_yaw_joint:    [-2.7576, 2.7576] rad
left_knee_joint:       [-0.087267, 2.8798] rad
left_ankle_pitch_joint: [-0.87267, 0.5236] rad
left_ankle_roll_joint:  [-0.2618, 0.2618] rad

// 躯干关节
waist_yaw_joint:       [-2.618, 2.618] rad
waist_roll_joint:      [-0.52, 0.52] rad
waist_pitch_joint:     [-0.52, 0.52] rad

// 手臂关节
shoulder_pitch_joint:  [-3.0892, 2.6704] rad
shoulder_roll_joint:   [-1.5882, 2.2515] rad
shoulder_yaw_joint:    [-2.618, 2.618] rad
elbow_joint:           [-1.0472, 2.0944] rad
wrist_roll_joint:      [-1.97222, 1.97222] rad
wrist_pitch_joint:     [-1.61443, 1.61443] rad
wrist_yaw_joint:       [-1.61443, 1.61443] rad
```

### 3. dq - 目标速度

- **单位**：弧度/秒（rad/s）
- **作用**：设置电机期望的角速度
- **注意**：在位置控制模式下，此参数通常设为0，让电机自然减速到目标位置

### 4. tau - 目标力矩

- **单位**：牛·米（N·m）
- **作用**：设置电机的期望输出力矩
- **注意**：在位置控制模式下，此参数通常设为0，让PID控制器自动计算所需力矩

### 5. kp - 位置增益

- **单位**：无量纲
- **作用**：控制位置误差的响应强度
- **影响**：值越大，电机越快地响应位置误差，但可能导致震荡

**推荐值**：
```cpp
// 根据关节类型设置不同的增益
腿部关节（0-11）:    kp = 100.0
躯干关节（12-14）:   kp = 50.0
手臂关节（15-28）:   kp = 50.0
```

### 6. kd - 速度增益

- **单位**：无量纲
- **作用**：控制速度误差的响应强度，提供阻尼效果
- **影响**：值越大，系统越稳定，但响应可能变慢

**推荐值**：
```cpp
所有关节: kd = 1.0
```

## 控制原理

### PID控制算法

宇树电机采用PID控制算法：

```
输出力矩 = kp * (目标位置 - 当前位置) + kd * (目标速度 - 当前速度) + 前馈力矩
```

其中：
- `kp * (目标位置 - 当前位置)`：位置误差项
- `kd * (目标速度 - 当前速度)`：速度误差项
- `前馈力矩`：tau参数

### 控制模式详解

#### 1. 位置控制模式（推荐）

```cpp
// 设置电机到指定角度
low_cmd.motor_cmd[i].mode = 1;        // 位置控制
low_cmd.motor_cmd[i].q = 0.5;         // 目标角度0.5弧度
low_cmd.motor_cmd[i].dq = 0.0;        // 目标速度为0
low_cmd.motor_cmd[i].tau = 0.0;       // 前馈力矩为0
low_cmd.motor_cmd[i].kp = 100.0;      // 位置增益
low_cmd.motor_cmd[i].kd = 1.0;        // 速度增益
```

#### 2. 速度控制模式

```cpp
// 设置电机以指定速度旋转
low_cmd.motor_cmd[i].mode = 2;        // 速度控制
low_cmd.motor_cmd[i].q = 0.0;         // 位置参数无效
low_cmd.motor_cmd[i].dq = 0.1;        // 目标速度0.1弧度/秒
low_cmd.motor_cmd[i].tau = 0.0;       // 前馈力矩
```

#### 3. 力矩控制模式

```cpp
// 设置电机输出指定力矩
low_cmd.motor_cmd[i].mode = 3;        // 力矩控制
low_cmd.motor_cmd[i].q = 0.0;         // 位置参数无效
low_cmd.motor_cmd[i].dq = 0.0;        // 速度参数无效
low_cmd.motor_cmd[i].tau = 5.0;       // 目标力矩5牛·米
```

## 实际应用示例

### 1. 机器人站立姿态

```cpp
void setStandingPose(unitree_hg::msg::LowCmd& low_cmd) {
    // 初始化所有电机
    for (int i = 0; i < 29; i++) {
        low_cmd.motor_cmd[i].mode = 1;
        low_cmd.motor_cmd[i].q = 0.0;      // 所有关节角度为0
        low_cmd.motor_cmd[i].dq = 0.0;
        low_cmd.motor_cmd[i].tau = 0.0;
        low_cmd.motor_cmd[i].kp = getPositionGain(i);
        low_cmd.motor_cmd[i].kd = 1.0;
    }
    
    // 设置腿部关节为站立角度
    // 左腿
    low_cmd.motor_cmd[0].q = 0.0;      // 左髋俯仰
    low_cmd.motor_cmd[1].q = 0.0;      // 左髋横滚
    low_cmd.motor_cmd[2].q = 0.0;      // 左髋偏航
    low_cmd.motor_cmd[3].q = 0.0;      // 左膝
    low_cmd.motor_cmd[4].q = 0.0;      // 左踝俯仰
    low_cmd.motor_cmd[5].q = 0.0;      // 左踝横滚
    
    // 右腿
    low_cmd.motor_cmd[6].q = 0.0;      // 右髋俯仰
    low_cmd.motor_cmd[7].q = 0.0;      // 右髋横滚
    low_cmd.motor_cmd[8].q = 0.0;      // 右髋偏航
    low_cmd.motor_cmd[9].q = 0.0;      // 右膝
    low_cmd.motor_cmd[10].q = 0.0;     // 右踝俯仰
    low_cmd.motor_cmd[11].q = 0.0;     // 右踝横滚
    
    // 躯干
    low_cmd.motor_cmd[12].q = 0.0;     // 腰部偏航
    low_cmd.motor_cmd[13].q = 0.0;     // 腰部横滚
    low_cmd.motor_cmd[14].q = 0.0;     // 腰部俯仰
    
    // 手臂自然下垂
    for (int i = 15; i < 29; i++) {
        low_cmd.motor_cmd[i].q = 0.0;
    }
}
```

### 2. 动补数据转换

```cpp
void convertMotionCaptureToMotorCmd(
    const std::map<std::string, double>& joint_angles,
    unitree_hg::msg::LowCmd& low_cmd) {
    
    // 关节映射表
    std::map<std::string, int> joint_mapping = {
        {"left_hip_pitch_joint", 0},
        {"left_hip_roll_joint", 1},
        {"left_hip_yaw_joint", 2},
        {"left_knee_joint", 3},
        {"left_ankle_pitch_joint", 4},
        {"left_ankle_roll_joint", 5},
        // ... 其他关节映射
    };
    
    // 初始化所有电机
    for (int i = 0; i < 29; i++) {
        low_cmd.motor_cmd[i].mode = 1;
        low_cmd.motor_cmd[i].q = 0.0;
        low_cmd.motor_cmd[i].dq = 0.0;
        low_cmd.motor_cmd[i].tau = 0.0;
        low_cmd.motor_cmd[i].kp = getPositionGain(i);
        low_cmd.motor_cmd[i].kd = 1.0;
    }
    
    // 设置关节角度
    for (const auto& joint : joint_angles) {
        std::string joint_name = joint.first;
        double angle = joint.second;
        
        if (joint_mapping.find(joint_name) != joint_mapping.end()) {
            int motor_index = joint_mapping[joint_name];
            
            // 角度限制
            angle = clampToJointLimits(joint_name, angle);
            
            // 设置目标位置
            low_cmd.motor_cmd[motor_index].q = static_cast<float>(angle);
        }
    }
}
```

### 3. 平滑运动控制

```cpp
class SmoothMotionController {
private:
    std::map<int, double> current_angles_;
    std::map<int, double> target_angles_;
    double max_velocity_ = 1.0;  // 最大角速度（弧度/秒）
    
public:
    void updateMotorCommands(unitree_hg::msg::LowCmd& low_cmd) {
        for (int i = 0; i < 29; i++) {
            // 计算平滑过渡
            double angle_diff = target_angles_[i] - current_angles_[i];
            double max_change = max_velocity_ * 0.005;  // 5ms时间步长
            
            if (abs(angle_diff) > max_change) {
                current_angles_[i] += (angle_diff > 0 ? max_change : -max_change);
            } else {
                current_angles_[i] = target_angles_[i];
            }
            
            // 设置电机命令
            low_cmd.motor_cmd[i].mode = 1;
            low_cmd.motor_cmd[i].q = static_cast<float>(current_angles_[i]);
            low_cmd.motor_cmd[i].dq = 0.0;
            low_cmd.motor_cmd[i].tau = 0.0;
            low_cmd.motor_cmd[i].kp = getPositionGain(i);
            low_cmd.motor_cmd[i].kd = 1.0;
        }
    }
    
    void setTargetAngles(const std::map<int, double>& target_angles) {
        target_angles_ = target_angles;
    }
};
```

## 安全注意事项

### 1. 角度限制

```cpp
double clampToJointLimits(const std::string& joint_name, double angle) {
    static std::map<std::string, std::pair<double, double>> limits = {
        {"left_hip_pitch_joint", {-2.5307, 2.8798}},
        {"left_hip_roll_joint", {-0.5236, 2.9671}},
        // ... 其他关节限制
    };
    
    if (limits.find(joint_name) != limits.end()) {
        auto limit = limits[joint_name];
        return std::clamp(angle, limit.first, limit.second);
    }
    
    return angle;
}
```

### 2. 紧急停止

```cpp
void emergencyStop(unitree_hg::msg::LowCmd& low_cmd) {
    for (int i = 0; i < 29; i++) {
        low_cmd.motor_cmd[i].mode = 0;  // 停止模式
        low_cmd.motor_cmd[i].q = 0.0;
        low_cmd.motor_cmd[i].dq = 0.0;
        low_cmd.motor_cmd[i].tau = 0.0;
        low_cmd.motor_cmd[i].kp = 0.0;
        low_cmd.motor_cmd[i].kd = 0.0;
    }
}
```

### 3. 渐进式启动

```cpp
void gradualStart(unitree_hg::msg::LowCmd& low_cmd, double progress) {
    for (int i = 0; i < 29; i++) {
        low_cmd.motor_cmd[i].mode = 1;
        low_cmd.motor_cmd[i].q = static_cast<float>(progress * target_angle);
        low_cmd.motor_cmd[i].dq = 0.0;
        low_cmd.motor_cmd[i].tau = 0.0;
        low_cmd.motor_cmd[i].kp = progress * getPositionGain(i);
        low_cmd.motor_cmd[i].kd = 1.0;
    }
}
```

## 调试技巧

### 1. 单关节测试

```cpp
void testSingleJoint(int motor_index, double angle) {
    unitree_hg::msg::LowCmd low_cmd;
    
    // 初始化所有电机为停止状态
    for (int i = 0; i < 29; i++) {
        low_cmd.motor_cmd[i].mode = 0;
    }
    
    // 只控制指定电机
    low_cmd.motor_cmd[motor_index].mode = 1;
    low_cmd.motor_cmd[motor_index].q = static_cast<float>(angle);
    low_cmd.motor_cmd[motor_index].dq = 0.0;
    low_cmd.motor_cmd[motor_index].tau = 0.0;
    low_cmd.motor_cmd[motor_index].kp = 50.0;  // 较小的增益
    low_cmd.motor_cmd[motor_index].kd = 1.0;
    
    // 发布命令
    lowcmd_publisher_->publish(low_cmd);
}
```

### 2. 参数调优

```cpp
void tunePIDParameters(int motor_index) {
    // 从较小的增益开始
    float kp_values[] = {10.0, 25.0, 50.0, 100.0, 200.0};
    float kd_values[] = {0.5, 1.0, 2.0, 5.0};
    
    for (float kp : kp_values) {
        for (float kd : kd_values) {
            // 测试不同的参数组合
            testMotorWithParams(motor_index, kp, kd);
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
    }
}
```

## 总结

宇树G1机器人的电机控制是一个复杂的系统，需要理解每个参数的含义和作用。通过合理设置电机命令参数，可以实现精确的机器人控制。在实际应用中，建议：

1. 使用位置控制模式作为主要控制方式
2. 根据关节类型设置合适的增益参数
3. 始终进行角度限制检查
4. 实现平滑运动过渡
5. 添加安全保护机制

通过本文档的指导，你应该能够有效地控制宇树G1机器人的29个电机，实现各种复杂的动作和任务。
