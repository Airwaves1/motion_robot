# Motion Robot - 精简版

## 项目概述

这是一个基于ROS2的动捕机器人控制系统，用于将VRPN动捕数据实时映射到Unitree G1机器人上。

## 主要功能

- **VRPN数据接收**: 订阅29个VRPN动捕传感器话题
- **实时控制**: 500Hz高频电机控制
- **状态监控**: 发布系统运行状态信息

## 发布的话题

1. `/motion_robot/status` - 系统状态信息 (1Hz)
2. `/lowcmd` - G1机器人电机控制命令 (500Hz)

## 构建和运行

```bash
# 构建项目
colcon build --packages-select motion_robot

# 运行主程序
ros2 run motion_robot motion_robot_node
```

## 配置参数

- `vrpn_tracker_name`: VRPN跟踪器名称 (默认: "MCServer")
- `sensor_id_offset`: 传感器ID偏移量 (默认: 302)
- `num_sensors`: 传感器数量 (默认: 29)
- `print_frequency`: 状态打印频率 (默认: 1.0 Hz)

## 项目结构

```
motion_robot/
├── src/
│   ├── motion_robot.cpp      # 主程序入口
│   ├── topic_data_getter.cpp # VRPN数据获取
│   ├── g1_controller.cpp     # G1机器人控制
│   └── motor_crc.cpp         # CRC校验
├── include/motion_robot/     # 头文件
├── launch/                   # 启动文件
└── CMakeLists.txt           # 构建配置
```

## 精简说明

- 移除了所有测试代码和调试日志
- **静默运行**: 移除了所有INFO、WARN、DEBUG级别的日志输出
- 只保留ERROR级别的关键错误信息
- 优化了代码结构，提高了运行效率
- 大幅减少了日志噪音，系统运行更加安静
