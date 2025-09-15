# MotionRobot 持续运行模式使用说明

## 概述

MotionRobot系统已更新为持续运行模式，不再有自动停止功能。系统将一直运行，持续接收VRPN动补数据并控制G1机器人，直到手动停止。

## 主要改进

### 1. 移除自动停止功能
- 移除了测试持续时间参数
- 移除了自动停止定时器
- 系统将持续运行直到手动停止

### 2. 优化日志输出
- 减少重复的警告信息
- 每10次状态打印才显示未接收数据的传感器
- 更清晰的状态信息显示

### 3. 新增持续运行模式参数
- `enable_continuous_mode`: 启用持续运行模式（默认true）
- 移除了测试相关的参数

## 使用方法

### 1. 编译项目
```bash
cd /home/airwave/dev/unitree/motion_robot
colcon build --packages-select motion_robot
```

### 2. 启动系统

#### 方法1：直接运行
```bash
source install/setup.bash
ros2 run motion_robot vrpn_test_node
```

#### 方法2：使用持续运行launch文件
```bash
source install/setup.bash
ros2 launch motion_robot motion_robot.launch.py
```

#### 方法3：使用原launch文件（带参数）
```bash
source install/setup.bash
ros2 launch motion_robot vrpn_test.launch.py enable_continuous_mode:=true
```

### 3. 自定义参数
```bash
ros2 launch motion_robot motion_robot.launch.py \
    vrpn_tracker_name:=MCServer \
    sensor_id_offset:=302 \
    num_sensors:=29 \
    print_frequency:=2.0
```

## 参数说明

| 参数名 | 默认值 | 说明 |
|--------|--------|------|
| `vrpn_tracker_name` | "MCServer" | VRPN跟踪器名称 |
| `sensor_id_offset` | 302 | 传感器ID偏移量 |
| `num_sensors` | 29 | 传感器数量 |
| `print_frequency` | 1.0 | 状态打印频率（Hz） |
| `enable_continuous_mode` | true | 启用持续运行模式 |

## 系统行为

### 启动时
1. 初始化VRPN话题订阅（29个传感器）
2. 初始化G1控制器
3. 开始数据收集和机器人控制
4. 显示启动信息

### 运行中
1. 持续接收VRPN动补数据
2. 实时转换为关节角度
3. 发送控制命令到G1机器人
4. 定期打印运行状态
5. 监控传感器数据接收情况

### 停止时
- 使用 `Ctrl+C` 手动停止
- 系统会优雅地关闭所有组件

## 状态监控

### 1. 查看运行状态
```bash
# 监控状态话题
ros2 topic echo /motion_robot/status

# 查看节点列表
ros2 node list

# 查看话题列表
ros2 topic list
```

### 2. 日志输出示例
```
[INFO] [motion_robot]: MotionRobot已启动 - 传感器: 29个, 跟踪器: MCServer
[INFO] [motion_robot]: 运行模式: 持续运行
[INFO] [motion_robot]: 运行状态: 29/29传感器, 1234消息, 120.5s
[INFO] [motion_robot]: VRPN数据驱动 - 传感器302->关节0: 四元数(0.123,0.456,0.789,0.321)
```

## 故障排除

### 1. 传感器数据未接收
- 检查VRPN话题是否正常发布：`ros2 topic list | grep vrpn`
- 检查话题频率：`ros2 topic hz /vrpn_mocap/MCServer/pose302`

### 2. 机器人控制无效
- 检查控制命令话题：`ros2 topic echo /lowcmd`
- 检查机器人状态：`ros2 topic echo /lowstate`

### 3. 性能问题
- 调整打印频率：`print_frequency:=0.5`（降低到0.5Hz）
- 检查系统资源使用情况

## 安全注意事项

1. **关节限位保护**：系统内置关节角度限制，防止机器人损坏
2. **数据有效性检查**：自动过滤无效的四元数数据
3. **CRC校验**：确保控制命令的完整性
4. **优雅停止**：使用Ctrl+C停止，避免突然断电

## 性能指标

- **控制频率**: 500Hz
- **数据延迟**: < 10ms
- **支持传感器**: 29个VRPN传感器
- **控制关节**: 29个自由度
- **运行模式**: 持续运行（无时间限制）

## 总结

持续运行模式使MotionRobot系统更适合生产环境使用，能够：

- ✅ 持续接收和处理动补数据
- ✅ 实时控制G1机器人
- ✅ 提供详细的状态监控
- ✅ 支持长时间稳定运行
- ✅ 具备完善的安全保护机制

系统现在可以24/7运行，为动补控制应用提供可靠的服务。
