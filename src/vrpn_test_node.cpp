#include "motion_robot/topic_data_getter.hpp"
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <iomanip>

using namespace motion_robot;

/**
 * @brief VRPN测试节点
 * 
 * 简单的VRPN数据获取和测试程序
 */
class VrpnTestNode : public rclcpp::Node {
public:
    VrpnTestNode() : Node("vrpn_test_node") {
        // 声明参数
        this->declare_parameter("tracker_name", "MCServer");
        this->declare_parameter("sensor_id_offset", 300);
        this->declare_parameter("num_sensors", 29);
        this->declare_parameter("print_frequency", 1.0); // 打印频率（Hz）
        this->declare_parameter("test_duration", 30.0);  // 测试持续时间（秒）
        
        // 获取参数
        std::string tracker_name = this->get_parameter("tracker_name").as_string();
        int sensor_id_offset = this->get_parameter("sensor_id_offset").as_int();
        int num_sensors = this->get_parameter("num_sensors").as_int();
        double print_frequency = this->get_parameter("print_frequency").as_double();
        double test_duration = this->get_parameter("test_duration").as_double();
        
        // 创建数据获取器
        data_getter_ = std::make_shared<TopicDataGetter>(
            "vrpn_data_getter", tracker_name, sensor_id_offset, num_sensors
        );
        
        // 设置数据回调
        data_getter_->setDataCallback([this](int sensor_id, const geometry_msgs::msg::Pose& pose, float joint_angle) {
            this->onVrpnData(sensor_id, pose, joint_angle);
        });
        
        // 创建定时器用于定期打印状态
        print_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / print_frequency)),
            [this]() { this->printStatus(); }
        );
        
        // 创建测试结束定时器
        if (test_duration > 0) {
            test_timer_ = this->create_wall_timer(
                std::chrono::milliseconds(static_cast<int>(test_duration * 1000)),
                [this]() { this->endTest(); }
            );
        }
        
        start_time_ = this->get_clock()->now();
        
        RCLCPP_INFO(this->get_logger(), "VRPN测试节点已启动");
        RCLCPP_INFO(this->get_logger(), "跟踪器名称: %s", tracker_name.c_str());
        RCLCPP_INFO(this->get_logger(), "传感器数量: %d", num_sensors);
        RCLCPP_INFO(this->get_logger(), "测试持续时间: %.1f 秒", test_duration);
    }
    
    ~VrpnTestNode() = default;

private:
    // 数据获取器
    std::shared_ptr<TopicDataGetter> data_getter_;
    
    // 定时器
    rclcpp::TimerBase::SharedPtr print_timer_;
    rclcpp::TimerBase::SharedPtr test_timer_;
    
    // 测试状态
    rclcpp::Time start_time_;
    bool test_ended_ = false;
    
    // 数据统计
    std::map<int, int> sensor_data_counts_;
    std::map<int, float> last_joint_angles_;
    
    // VRPN数据回调
    void onVrpnData(int sensor_id, const geometry_msgs::msg::Pose& pose, float joint_angle) {
        sensor_data_counts_[sensor_id]++;
        last_joint_angles_[sensor_id] = joint_angle;
        
        // 每10次数据打印一次详细信息
        if (sensor_data_counts_[sensor_id] % 10 == 0) {
            RCLCPP_INFO(this->get_logger(),
                "传感器 %d: 位置(%.2f,%.2f,%.2f) 四元数(%.3f,%.3f,%.3f,%.3f) -> 关节角度: %.3f",
                sensor_id,
                pose.position.x, pose.position.y, pose.position.z,
                pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w,
                joint_angle);
        }
    }
    
    // 打印状态信息
    void printStatus() {
        if (test_ended_) return;
        
        auto current_time = this->get_clock()->now();
        double elapsed_time = (current_time - start_time_).seconds();
        
        // 获取统计信息
        auto stats = data_getter_->getStatistics();
        
        // 发布状态消息
        auto status_msg = std_msgs::msg::String();
        status_msg.data = "VRPN测试 - 活跃传感器: " + std::to_string(stats.active_sensors) + 
                         "/" + std::to_string(29) + 
                         ", 总消息数: " + std::to_string(stats.total_messages) +
                         ", 运行时间: " + std::to_string(static_cast<int>(elapsed_time)) + "s";
        
        RCLCPP_INFO(this->get_logger(), "=== VRPN测试状态 ===");
        RCLCPP_INFO(this->get_logger(), "运行时间: %.1f 秒", elapsed_time);
        RCLCPP_INFO(this->get_logger(), "活跃传感器: %d/%d", stats.active_sensors, 29);
        RCLCPP_INFO(this->get_logger(), "总消息数: %d", stats.total_messages);
        
        if (stats.active_sensors > 0) {
            RCLCPP_INFO(this->get_logger(), "接收率: %.1f%%", 
                       (double)stats.active_sensors / 29 * 100.0);
        }
        
        // 打印前几个传感器的状态
        RCLCPP_INFO(this->get_logger(), "传感器详细状态:");
        int count = 0;
        for (const auto& [sensor_id, data_count] : sensor_data_counts_) {
            if (count >= 5) break; // 只显示前5个
            
            double message_rate = data_count / elapsed_time;
            RCLCPP_INFO(this->get_logger(), 
                       "  传感器 %d: %d 消息, %.1f Hz, 关节角度: %.3f", 
                       sensor_id, data_count, message_rate, last_joint_angles_[sensor_id]);
            count++;
        }
        RCLCPP_INFO(this->get_logger(), "========================");
    }
    
    // 结束测试
    void endTest() {
        if (test_ended_) return;
        
        test_ended_ = true;
        print_timer_->cancel();
        
        RCLCPP_INFO(this->get_logger(), "=== VRPN测试完成 ===");
        
        // 最终统计
        auto stats = data_getter_->getStatistics();
        RCLCPP_INFO(this->get_logger(), "最终结果:");
        RCLCPP_INFO(this->get_logger(), "  活跃传感器: %d/%d", stats.active_sensors, 29);
        RCLCPP_INFO(this->get_logger(), "  总消息数: %d", stats.total_messages);
        RCLCPP_INFO(this->get_logger(), "  接收率: %.1f%%", 
                   (double)stats.active_sensors / 29 * 100.0);
        
        if (stats.active_sensors == 29) {
            RCLCPP_INFO(this->get_logger(), "✅ 所有传感器都成功接收到数据！");
        } else {
            RCLCPP_WARN(this->get_logger(), "⚠️  有 %d 个传感器未接收到数据", 
                       29 - stats.active_sensors);
        }
        
        RCLCPP_INFO(this->get_logger(), "========================");
        
        // 退出程序
        rclcpp::shutdown();
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<VrpnTestNode>();
    
    try {
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("vrpn_test_node"), "异常: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}
