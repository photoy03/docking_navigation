#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <sstream>
#include <cstring>
#include <cstdio>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "CSerialPort/SerialPort.h"

using namespace std::chrono_literals;
using namespace itas109;

class MCUBridge : public rclcpp::Node
{
public:
    MCUBridge()
    : Node("mcu_bridge_node"),
      read_buffer_(""),
      mcu_state_(-1),
      docked_(false),
      emergency_(false),
      emergency_tx_pending_(false)
    {
        this->declare_parameter<std::string>("port_name", "/dev/arduino");
        this->declare_parameter<int>("baud_rate", 115200);

        const std::string port_name = this->get_parameter("port_name").as_string();
        const int baud_rate = this->get_parameter("baud_rate").as_int();

        sp_.init(port_name.c_str(), baud_rate, ParityNone, DataBits8, StopOne);

        if (sp_.open()) {
            RCLCPP_INFO(this->get_logger(), "MCU 시리얼 연결 성공: %s", port_name.c_str());
        } else {
            RCLCPP_ERROR(this->get_logger(), "시리얼 포트 열기 실패: %s", port_name.c_str());
        }

        auto state_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();

        tick_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>(
            "/robot1/wheel_ticks", 10);
        state_pub_ = this->create_publisher<std_msgs::msg::Int32>(
            "/mcu/state", state_qos);
        docked_pub_ = this->create_publisher<std_msgs::msg::Bool>(
            "/mcu/docked", state_qos);
        emergency_pub_ = this->create_publisher<std_msgs::msg::Bool>(
            "/mcu/emergency", state_qos);

        cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&MCUBridge::cmd_callback, this, std::placeholders::_1));

        mode_cmd_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "/mcu/mode_cmd", 10,
            std::bind(&MCUBridge::mode_cmd_callback, this, std::placeholders::_1));

        emergency_cmd_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/mcu/emergency_cmd", 10,
            std::bind(&MCUBridge::emergency_cmd_callback, this, std::placeholders::_1));

        read_timer_ = this->create_wall_timer(
            10ms,
            std::bind(&MCUBridge::read_serial_callback, this));

        RCLCPP_INFO(this->get_logger(), "========================================");
        RCLCPP_INFO(this->get_logger(), "MCU Bridge START");
        RCLCPP_INFO(this->get_logger(), "Protocol: M / V / E / D / S / T / B");
        RCLCPP_INFO(this->get_logger(), "Emergency recovery E,0 지원");
        RCLCPP_INFO(this->get_logger(), "Emergency ACK 분리: pending / confirmed");
        RCLCPP_INFO(this->get_logger(), "========================================");
    }

    ~MCUBridge()
    {
        if (sp_.isOpen()) {
            sp_.close();
        }
    }

private:
    std::vector<std::string> split_csv(const std::string & line)
    {
        std::stringstream ss(line);
        std::string token;
        std::vector<std::string> tokens;

        while (std::getline(ss, token, ',')) {
            tokens.push_back(token);
        }

        return tokens;
    }

    void cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        if (!sp_.isOpen()) {
            return;
        }

        // E,1 송신 직후부터 STM ACK 전에도 cmd_vel 차단
        if (emergency_ || emergency_tx_pending_) {
            return;
        }

        // STM이 실제 STATE2(AUTO)를 보고한 경우에만 V 명령 송신
        if (mcu_state_ != 2) {
            return;
        }

        char buffer[64];
        std::snprintf(
            buffer,
            sizeof(buffer),
            "V,%.6f,%.6f\n",
            msg->linear.x,
            msg->angular.z);

        sp_.writeData(buffer, std::strlen(buffer));
    }

    void mode_cmd_callback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        if (!sp_.isOpen()) {
            RCLCPP_WARN(this->get_logger(), "MCU mode 명령 실패: Serial port closed");
            return;
        }

        const int requested_mode = msg->data;

        if (requested_mode < 0 || requested_mode > 2) {
            RCLCPP_WARN(this->get_logger(), "잘못된 MCU mode 요청: %d", requested_mode);
            return;
        }

        // Emergency confirmed 또는 pending 중에는 M1/M2 차단. M0만 허용.
        if ((emergency_ || emergency_tx_pending_) && requested_mode != 0) {
            RCLCPP_ERROR(
                this->get_logger(),
                "Emergency 상태/pending: M,%d 요청 차단",
                requested_mode);
            return;
        }

        char buffer[16];
        std::snprintf(buffer, sizeof(buffer), "M,%d\n", requested_mode);
        sp_.writeData(buffer, std::strlen(buffer));

        RCLCPP_INFO(this->get_logger(), "[MCU TX] M,%d", requested_mode);
    }

    void emergency_cmd_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (!msg->data) {
            return;
        }

        if (!sp_.isOpen()) {
            RCLCPP_ERROR(this->get_logger(), "Emergency 명령 실패: Serial port closed");
            return;
        }

        // 중복 Emergency 요청 방지
        if (emergency_ || emergency_tx_pending_) {
            RCLCPP_WARN(
                this->get_logger(),
                "Emergency 요청 무시: 이미 confirmed 또는 pending");
            return;
        }

        const char * command = "E,1\n";
        sp_.writeData(command, std::strlen(command));

        // 여기서는 /mcu/emergency=true를 publish하지 않는다.
        // STM의 E,1,<reason> 응답을 받은 뒤 confirmed 처리한다.
        emergency_tx_pending_ = true;

        RCLCPP_ERROR(
            this->get_logger(),
            "[MCU TX] E,1 - Emergency 요청 (ACK 대기)");
    }

    void read_serial_callback()
    {
        if (!sp_.isOpen()) {
            return;
        }

        char temp_buf[256];
        const int read_len = sp_.readData(temp_buf, sizeof(temp_buf) - 1);

        if (read_len <= 0) {
            return;
        }

        temp_buf[read_len] = '\0';
        read_buffer_ += temp_buf;

        size_t pos;
        while ((pos = read_buffer_.find('\n')) != std::string::npos) {
            std::string line = read_buffer_.substr(0, pos);
            read_buffer_.erase(0, pos + 1);

            if (!line.empty() && line.back() == '\r') {
                line.pop_back();
            }

            if (line.empty()) {
                continue;
            }

            parse_serial_line(line);
        }
    }

    void parse_serial_line(const std::string & line)
    {
        const auto tokens = split_csv(line);

        if (tokens.empty()) {
            return;
        }

        // B,READY / B,TARE_OK
        if (tokens[0] == "B") {
            if (tokens.size() < 2) {
                return;
            }

            const std::string boot_msg = tokens[1];
            RCLCPP_INFO(this->get_logger(), "[MCU BOOT] %s", boot_msg.c_str());

            if (boot_msg == "READY") {
                mcu_state_ = 0;
                docked_ = false;
                emergency_ = false;
                emergency_tx_pending_ = false;

                std_msgs::msg::Int32 state_msg;
                state_msg.data = 0;
                state_pub_->publish(state_msg);

                std_msgs::msg::Bool dock_msg;
                dock_msg.data = false;
                docked_pub_->publish(dock_msg);

                std_msgs::msg::Bool emergency_msg;
                emergency_msg.data = false;
                emergency_pub_->publish(emergency_msg);
            }

            return;
        }

        // D,1 = first docking complete
        if (tokens[0] == "D") {
            if (tokens.size() < 2) {
                return;
            }

            try {
                const int value = std::stoi(tokens[1]);
                docked_ = (value == 1);

                std_msgs::msg::Bool msg;
                msg.data = docked_;
                docked_pub_->publish(msg);

                RCLCPP_INFO(
                    this->get_logger(),
                    "[MCU DOCK] docked=%s",
                    docked_ ? "true" : "false");
            } catch (...) {
                RCLCPP_WARN(this->get_logger(), "잘못된 Dock 패킷: %s", line.c_str());
            }

            return;
        }

        // S,0 STOP/EMERGENCY / S,1 ADMITTANCE / S,2 AUTO
        if (tokens[0] == "S") {
            if (tokens.size() < 2) {
                return;
            }

            try {
                const int state = std::stoi(tokens[1]);

                if (state < 0 || state > 2) {
                    RCLCPP_WARN(this->get_logger(), "알 수 없는 MCU state: %d", state);
                    return;
                }

                mcu_state_ = state;

                std_msgs::msg::Int32 msg;
                msg.data = mcu_state_;
                state_pub_->publish(msg);

                const char * state_name = "UNKNOWN";
                if (state == 0) {
                    state_name = "STOP";
                } else if (state == 1) {
                    state_name = "ADMITTANCE";
                } else if (state == 2) {
                    state_name = "AUTO";
                }

                RCLCPP_INFO(
                    this->get_logger(),
                    "[MCU STATE] %d (%s)",
                    state,
                    state_name);
            } catch (...) {
                RCLCPP_WARN(this->get_logger(), "잘못된 State 패킷: %s", line.c_str());
            }

            return;
        }

        // T,left_total,right_total
        if (tokens[0] == "T") {
            if (tokens.size() < 3) {
                return;
            }

            try {
                const int left_tick = std::stoi(tokens[1]);
                const int right_tick = std::stoi(tokens[2]);

                auto tick_msg = std_msgs::msg::Int32MultiArray();
                tick_msg.data.push_back(left_tick);
                tick_msg.data.push_back(right_tick);
                tick_pub_->publish(tick_msg);
            } catch (...) {
                RCLCPP_WARN(this->get_logger(), "손상된 Tick 패킷: %s", line.c_str());
            }

            return;
        }

        // E,1,<reason> / E,0
        if (tokens[0] == "E") {
            if (tokens.size() < 2) {
                return;
            }

            try {
                const int value = std::stoi(tokens[1]);

                if (value == 1) {
                    emergency_ = true;
                    emergency_tx_pending_ = false;

                    std_msgs::msg::Bool msg;
                    msg.data = true;
                    emergency_pub_->publish(msg);

                    std::string reason = "UNKNOWN";
                    if (tokens.size() >= 3) {
                        reason = tokens[2];
                    }

                    RCLCPP_ERROR(
                        this->get_logger(),
                        "[MCU EMERGENCY SET] reason=%s",
                        reason.c_str());
                    return;
                }

                if (value == 0) {
                    emergency_ = false;
                    emergency_tx_pending_ = false;

                    std_msgs::msg::Bool msg;
                    msg.data = false;
                    emergency_pub_->publish(msg);

                    RCLCPP_INFO(this->get_logger(), "[MCU EMERGENCY CLEAR] E,0");
                    return;
                }

                RCLCPP_WARN(
                    this->get_logger(),
                    "알 수 없는 Emergency value: %d",
                    value);
            } catch (...) {
                RCLCPP_WARN(
                    this->get_logger(),
                    "잘못된 Emergency 패킷: %s",
                    line.c_str());
            }

            return;
        }

        if (line == "PONG") {
            RCLCPP_INFO(this->get_logger(), "[MCU] PONG");
            return;
        }

        // STM 임시 debug telemetry는 ROS 상태 packet으로 처리하지 않음
        if (tokens[0] == "L" || tokens[0] == "A") {
            return;
        }

        RCLCPP_DEBUG(
            this->get_logger(),
            "알 수 없는 MCU packet: %s",
            line.c_str());
    }

    CSerialPort sp_;
    std::string read_buffer_;

    int mcu_state_;
    bool docked_;

    // STM에서 E,1 ACK를 받아 확인된 실제 Emergency 상태
    bool emergency_;

    // Jetson이 E,1을 보냈지만 아직 STM ACK를 받지 못한 상태
    bool emergency_tx_pending_;

    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr tick_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr state_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr docked_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr emergency_pub_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr mode_cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_cmd_sub_;

    rclcpp::TimerBase::SharedPtr read_timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MCUBridge>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

