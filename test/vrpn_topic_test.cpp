#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>
#include <map>
#include <vector>
#include <iomanip>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/**
 * @brief VRPN话题监听测试程序
 * 
 * 用于测试是否成功监听到VRPN动补数据话题
 */
class VrpnTopicTest : public rclcpp::Node {
public:
    VrpnTopicTest() : Node("vrpn_topic_test") {
        // 声明参数
        this->declare_parameter("vrpn_tracker_name", "MCServer");
        this->declare_parameter("sensor_id_offset", 302);
        this->declare_parameter("num_sensors", 29);
        this->declare_parameter("test_duration", 30.0);  // 测试持续时间（秒）
        this->declare_parameter("print_frequency", 1.0); // 打印频率（Hz）
        
        // 获取参数
        vrpn_tracker_name_ = this->get_parameter("vrpn_tracker_name").as_string();
        sensor_id_offset_ = this->get_parameter("sensor_id_offset").as_int();
        num_sensors_ = this->get_parameter("num_sensors").as_int();
        test_duration_ = this->get_parameter("test_duration").as_double();
        print_frequency_ = this->get_parameter("print_frequency").as_double();
        
        // 初始化统计信息
        initializeStatistics();
        
        // 创建VRPN话题订阅
        createVrpnSubscriptions();
        
        // 创建状态发布器
        status_pub_ = this->create_publisher<std_msgs::msg::String>("/vrpn_test_status", 10);
        
        // 创建定时器用于定期打印状态
        status_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / print_frequency_)),
            [this]() { this->printStatus(); }
        );
        
        // 创建测试结束定时器
        test_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(test_duration_ * 1000)),
            [this]() { this->endTest(); }
        );
        
        RCLCPP_INFO(this->get_logger(), "VRPN话题测试程序已启动");
        RCLCPP_INFO(this->get_logger(), "跟踪器名称: %s", vrpn_tracker_name_.c_str());
        RCLCPP_INFO(this->get_logger(), "传感器数量: %d", num_sensors_);
        RCLCPP_INFO(this->get_logger(), "传感器ID范围: %d - %d", sensor_id_offset_, sensor_id_offset_ + num_sensors_ - 1);
        RCLCPP_INFO(this->get_logger(), "测试持续时间: %.1f 秒", test_duration_);
    }
    
    ~VrpnTopicTest() = default;

private:
    // 配置参数
    std::string vrpn_tracker_name_;
    int sensor_id_offset_;
    int num_sensors_;
    double test_duration_;
    double print_frequency_;
    
    // ROS2接口
    std::vector<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> vrpn_subs_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::TimerBase::SharedPtr status_timer_;
    rclcpp::TimerBase::SharedPtr test_timer_;
    
    // 统计信息
    struct SensorStats {
        int message_count = 0;
        rclcpp::Time last_message_time;
        rclcpp::Time first_message_time;
        bool has_received = false;
        double last_position[3] = {0.0, 0.0, 0.0};
        double last_orientation[4] = {0.0, 0.0, 0.0, 1.0};
    };
    
    std::map<int, SensorStats> sensor_stats_;
    rclcpp::Time test_start_time_;
    bool test_ended_ = false;
    
    // 将四元数转换为欧拉角（弧度）
    void quaternionToEuler(double qx, double qy, double qz, double qw, 
                          double& roll, double& pitch, double& yaw) {
        // 四元数转欧拉角 - 参考vrpn_g1_mapper.cpp的实现
        double sinr_cosp = 2 * (qw * qx + qy * qz);
        double cosr_cosp = 1 - 2 * (qx * qx + qy * qy);
        roll = std::atan2(sinr_cosp, cosr_cosp);
        
        double sinp = 2 * (qw * qy - qz * qx);
        if (std::abs(sinp) >= 1) {
            pitch = std::copysign(M_PI / 2, sinp);
        } else {
            pitch = std::asin(sinp);
        }
        
        double siny_cosp = 2 * (qw * qz + qx * qy);
        double cosy_cosp = 1 - 2 * (qy * qy + qz * qz);
        yaw = std::atan2(siny_cosp, cosy_cosp);
    }
    
    // 初始化统计信息
    void initializeStatistics() {
        test_start_time_ = this->get_clock()->now();
        for (int i = 0; i < num_sensors_; i++) {
            int sensor_id = sensor_id_offset_ + i;
            sensor_stats_[sensor_id] = SensorStats();
        }
    }
    
    // 创建VRPN话题订阅
    void createVrpnSubscriptions() {
        for (int i = 0; i < num_sensors_; i++) {
            int sensor_id = sensor_id_offset_ + i;
            std::string topic = "/vrpn_mocap/" + vrpn_tracker_name_ + "/pose" + std::to_string(sensor_id);
            
            auto sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                topic, rclcpp::SensorDataQoS(),
                [this, sensor_id](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                    this->onVrpnPose(msg, sensor_id);
                }
            );
            vrpn_subs_.push_back(sub);
            
            RCLCPP_DEBUG(this->get_logger(), "订阅话题: %s", topic.c_str());
        }
        
        RCLCPP_INFO(this->get_logger(), "已创建 %zu 个VRPN话题订阅", vrpn_subs_.size());
    }
    
    // VRPN pose回调函数
    void onVrpnPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg, int sensor_id) {
        auto& stats = sensor_stats_[sensor_id];
        
        // 更新统计信息
        stats.message_count++;
        stats.last_message_time = msg->header.stamp;
        
        if (!stats.has_received) {
            stats.first_message_time = msg->header.stamp;
            stats.has_received = true;
            RCLCPP_INFO(this->get_logger(), "传感器 %d 首次接收到数据", sensor_id);
        }
        
        // 保存最新的位置和姿态
        stats.last_position[0] = msg->pose.position.x;
        stats.last_position[1] = msg->pose.position.y;
        stats.last_position[2] = msg->pose.position.z;
        stats.last_orientation[0] = msg->pose.orientation.x;
        stats.last_orientation[1] = msg->pose.orientation.y;
        stats.last_orientation[2] = msg->pose.orientation.z;
        stats.last_orientation[3] = msg->pose.orientation.w;
    }
    
    // 打印状态信息
    void printStatus() {
        if (test_ended_) return;
        
        auto current_time = this->get_clock()->now();
        double elapsed_time = (current_time - test_start_time_).seconds();
        
        // 统计接收到的传感器数量
        int received_sensors = 0;
        int total_messages = 0;
        
        for (const auto& [sensor_id, stats] : sensor_stats_) {
            if (stats.has_received) {
                received_sensors++;
                total_messages += stats.message_count;
            }
        }
        
        // 发布状态消息
        auto status_msg = std_msgs::msg::String();
        status_msg.data = "VRPN测试 - 已接收传感器: " + std::to_string(received_sensors) + 
                         "/" + std::to_string(num_sensors_) + 
                         ", 总消息数: " + std::to_string(total_messages) +
                         ", 运行时间: " + std::to_string(static_cast<int>(elapsed_time)) + "s";
        status_pub_->publish(status_msg);
        
        // 打印详细状态
        RCLCPP_INFO(this->get_logger(), "=== VRPN话题测试状态 ===");
        RCLCPP_INFO(this->get_logger(), "运行时间: %.1f 秒", elapsed_time);
        RCLCPP_INFO(this->get_logger(), "接收传感器: %d/%d", received_sensors, num_sensors_);
        RCLCPP_INFO(this->get_logger(), "总消息数: %d", total_messages);
        
        if (received_sensors > 0) {
            RCLCPP_INFO(this->get_logger(), "接收率: %.1f%%", 
                       (double)received_sensors / num_sensors_ * 100.0);
        }
        
        // 打印每个传感器的状态
        RCLCPP_INFO(this->get_logger(), "传感器详细状态:");
        for (const auto& [sensor_id, stats] : sensor_stats_) {
            if (stats.has_received) {
                double message_rate = stats.message_count / elapsed_time;
                RCLCPP_INFO(this->get_logger(), 
                           "  传感器 %d: %d 消息, %.1f Hz", 
                           sensor_id, stats.message_count, message_rate);
                RCLCPP_INFO(this->get_logger(), 
                           "    位置(%.2f,%.2f,%.2f)", 
                           stats.last_position[0], stats.last_position[1], stats.last_position[2]);
                
                // 显示四元数
                RCLCPP_INFO(this->get_logger(), 
                           "    四元数(%.4f,%.4f,%.4f,%.4f)", 
                           stats.last_orientation[0], stats.last_orientation[1], 
                           stats.last_orientation[2], stats.last_orientation[3]);
                
                // 转换为欧拉角并显示
                double roll, pitch, yaw;
                quaternionToEuler(stats.last_orientation[0], stats.last_orientation[1], 
                                stats.last_orientation[2], stats.last_orientation[3], 
                                roll, pitch, yaw);
                RCLCPP_INFO(this->get_logger(), 
                           "    欧拉角(%.3f,%.3f,%.3f) 度", 
                           roll * 180.0 / M_PI, pitch * 180.0 / M_PI, yaw * 180.0 / M_PI);
            } else {
                RCLCPP_WARN(this->get_logger(), "  传感器 %d: 未接收到数据", sensor_id);
            }
        }
        RCLCPP_INFO(this->get_logger(), "========================");
    }
    
    // 结束测试
    void endTest() {
        if (test_ended_) return;
        
        test_ended_ = true;
        status_timer_->cancel();
        
        RCLCPP_INFO(this->get_logger(), "=== VRPN话题测试完成 ===");
        
        // 最终统计
        int received_sensors = 0;
        int total_messages = 0;
        
        for (const auto& [sensor_id, stats] : sensor_stats_) {
            if (stats.has_received) {
                received_sensors++;
                total_messages += stats.message_count;
            }
        }
        
        RCLCPP_INFO(this->get_logger(), "最终结果:");
        RCLCPP_INFO(this->get_logger(), "  接收传感器: %d/%d", received_sensors, num_sensors_);
        RCLCPP_INFO(this->get_logger(), "  总消息数: %d", total_messages);
        RCLCPP_INFO(this->get_logger(), "  接收率: %.1f%%", 
                   (double)received_sensors / num_sensors_ * 100.0);
        
        if (received_sensors == num_sensors_) {
            RCLCPP_INFO(this->get_logger(), "✅ 所有传感器都成功接收到数据！");
        } else {
            RCLCPP_WARN(this->get_logger(), "⚠️  有 %d 个传感器未接收到数据", 
                       num_sensors_ - received_sensors);
        }
        
        RCLCPP_INFO(this->get_logger(), "========================");
        
        // 退出程序
        rclcpp::shutdown();
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<VrpnTopicTest>();
    
    try {
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("vrpn_topic_test"), "异常: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}
