#include "motion_robot/motion_robot.hpp"

namespace motion_robot {

MotionRobot::MotionRobot() : Node("motion_robot") {
    // 声明参数
    this->declare_parameter("vrpn_tracker_name", "MCServer");
    this->declare_parameter("sensor_id_offset", 302);
    this->declare_parameter("num_sensors", 29);
    this->declare_parameter("print_frequency", 1.0); // 打印频率（Hz）
    this->declare_parameter("enable_continuous_mode", true); // 持续运行模式
    
    // 获取参数
    vrpn_tracker_name_ = this->get_parameter("vrpn_tracker_name").as_string();
    sensor_id_offset_ = this->get_parameter("sensor_id_offset").as_int();
    num_sensors_ = this->get_parameter("num_sensors").as_int();
    print_frequency_ = this->get_parameter("print_frequency").as_double();
    bool continuous_mode = this->get_parameter("enable_continuous_mode").as_bool();
    
    // 设置启动时间
    start_time_ = this->get_clock()->now();
    
    // 创建状态发布器
    status_pub_ = this->create_publisher<std_msgs::msg::String>("/motion_robot/status", 10);
    
    // 创建定时器用于定期打印状态
    status_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / print_frequency_)),
        [this]() { this->printStatus(); }
    );
    
    RCLCPP_INFO(this->get_logger(), "MotionRobot已启动 - 传感器: %d个, 跟踪器: %s", 
               num_sensors_, vrpn_tracker_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "运行模式: %s", continuous_mode ? "持续运行" : "测试模式");
    
    // 延迟初始化TopicDataGetter和G1控制器
    initializeDataGetter();
    initializeG1Controller();
}

void MotionRobot::initializeDataGetter() {
    // 创建TopicDataGetter实例
    data_getter_ = std::make_shared<TopicDataGetter>(this);
    
    // 初始化数据获取器
    if (!data_getter_->initialize(vrpn_tracker_name_, sensor_id_offset_, num_sensors_)) {
        RCLCPP_ERROR(this->get_logger(), "TopicDataGetter初始化失败");
        return;
    }
    
    // 设置启动时间
    data_getter_->setTestStartTime(start_time_);
    
    // 设置VRPN数据回调
    data_getter_->setDataCallback([this](int sensor_id, float joint_angle) {
        this->onVrpnData(sensor_id, joint_angle);
    });
    
    // 开始数据收集
    data_getter_->startDataCollection();
}

void MotionRobot::initializeG1Controller() {
    // 创建G1控制器实例
    g1_controller_ = std::make_shared<G1Controller>(this);
    
    // 初始化G1控制器
    if (!g1_controller_->initialize()) {
        RCLCPP_ERROR(this->get_logger(), "G1Controller初始化失败");
        return;
    }
    
    // 设置默认PID参数
    g1_controller_->setPIDParams(-1, 20.0f, 0.5f);
    
    // 启用所有电机
    g1_controller_->enableMotor(-1, true);
    
    // 启用手臂摇摆模拟（测试用，已关闭）
    // g1_controller_->enableArmSwingSimulation(true);
    
    RCLCPP_INFO(this->get_logger(), "G1Controller初始化完成");
}

void MotionRobot::onVrpnData(int sensor_id, float /* joint_angle */) {
    // 将VRPN数据传递给G1控制器
    if (g1_controller_) {
        // 获取对应的pose数据
        auto pose_data = data_getter_->getLatestPose(sensor_id);
        if (pose_data) {
            const auto& pose = pose_data->pose;
            
            // 将pose数据转换为四元数并传递给G1控制器
            int joint_id = sensor_id - sensor_id_offset_;
            bool success = g1_controller_->simulateVrpnPoseData(
                joint_id,  // 转换为0-28的关节ID
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w
            );
            
            // 添加调试信息 - 显示所有关节
            if (success) {
                RCLCPP_INFO(this->get_logger(), "VRPN数据驱动 - 传感器%d->关节%d: 四元数(%.3f,%.3f,%.3f,%.3f)", 
                            sensor_id, joint_id, pose.orientation.x, pose.orientation.y, 
                            pose.orientation.z, pose.orientation.w);
            } else {
                RCLCPP_WARN(this->get_logger(), "VRPN数据处理失败 - 传感器%d->关节%d", sensor_id, joint_id);
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "未找到传感器%d的pose数据", sensor_id);
        }
    }
}

void MotionRobot::printStatus() {
    auto current_time = this->get_clock()->now();
    double elapsed_time = (current_time - start_time_).seconds();
    
    // 从TopicDataGetter获取统计信息
    int received_sensors = data_getter_->getReceivedSensorCount();
    int total_messages = 0;
    
    // 计算总消息数
    for (int i = 0; i < num_sensors_; i++) {
        int sensor_id = sensor_id_offset_ + i;
        auto stats = data_getter_->getSensorStats(sensor_id);
        total_messages += stats.message_count;
    }
    
    // 发布状态消息
    auto status_msg = std_msgs::msg::String();
    status_msg.data = "MotionRobot运行中 - 已接收传感器: " + std::to_string(received_sensors) + 
                     "/" + std::to_string(num_sensors_) + 
                     ", 总消息数: " + std::to_string(total_messages) +
                     ", 运行时间: " + std::to_string(static_cast<int>(elapsed_time)) + "s";
    status_pub_->publish(status_msg);
    
    // 打印简要状态
    RCLCPP_INFO(this->get_logger(), "运行状态: %d/%d传感器, %d消息, %.1fs", 
               received_sensors, num_sensors_, total_messages, elapsed_time);
    
    // 只显示未接收数据的传感器（每10次打印一次，避免日志过多）
    static int print_count = 0;
    print_count++;
    if (print_count % 10 == 0) {
        bool has_missing_sensors = false;
        for (int i = 0; i < num_sensors_; i++) {
            int sensor_id = sensor_id_offset_ + i;
            auto stats = data_getter_->getSensorStats(sensor_id);
            if (!stats.has_received) {
                if (!has_missing_sensors) {
                    RCLCPP_WARN(this->get_logger(), "未接收数据的传感器:");
                    has_missing_sensors = true;
                }
                RCLCPP_WARN(this->get_logger(), "  传感器 %d", sensor_id);
            }
        }
    }
}

// endTest函数已移除 - 系统现在持续运行

} // namespace motion_robot
