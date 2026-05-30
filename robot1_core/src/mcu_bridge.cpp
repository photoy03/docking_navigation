#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

// CSerialPort 헤더 (가장 확실한 경로)
#include "CSerialPort/SerialPort.h"

using namespace std::chrono_literals;
using namespace itas109; // CSerialPort 네임스페이스

class MCUBridge : public rclcpp::Node
{
public:
    MCUBridge() : Node("mcu_bridge_node"), read_buffer_("")
    {
        // ROS 2 파라미터 선언 (기본 포트: /dev/ttyUSB0, 통신속도: 115200)
        this->declare_parameter<std::string>("port_name", "/dev/arduino");
        this->declare_parameter<int>("baud_rate", 115200);

        std::string port_name = this->get_parameter("port_name").as_string();
        int baud_rate = this->get_parameter("baud_rate").as_int();

        // 1. CSerialPort 초기화 및 연결
        sp_.init(port_name.c_str(), baud_rate, ParityNone, DataBits8, StopOne);
        
        if (sp_.open()) {
            RCLCPP_INFO(this->get_logger(), "MCU 시리얼 연결 성공: %s", port_name.c_str());
        } else {
            RCLCPP_ERROR(this->get_logger(), "시리얼 포트 열기 실패! 포트 권한(chmod 666)을 확인하세요.");
        }

        // 2. Publisher & Subscriber 세팅
        tick_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("/robot1/wheel_ticks", 10);
        cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&MCUBridge::cmd_callback, this, std::placeholders::_1));
            
        // 3. 10ms(100Hz) 주기로 시리얼 데이터를 읽어오는 타이머
        read_timer_ = this->create_wall_timer(10ms, std::bind(&MCUBridge::read_serial_callback, this));
    }

    ~MCUBridge()
    {
        // 노드 종료 시 안전하게 포트 닫기
        if (sp_.isOpen()) sp_.close();
    }

private:
    // [젯슨 -> 아두이노] cmd_vel(속도 명령)을 시리얼로 전송
    void cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        if (!sp_.isOpen()) return;

        char buffer[50];
        // 아두이노로 보낼 데이터 포맷: "V,선속도,각속도\n"
        snprintf(buffer, sizeof(buffer), "V,%.2f,%.2f\n", msg->linear.x, msg->angular.z);
        sp_.writeData(buffer, strlen(buffer)); 
    }

    // [아두이노 -> 젯슨] 엔코더 틱 데이터를 읽어서 ROS 토픽으로 발행
    void read_serial_callback()
    {
        if (!sp_.isOpen()) return;

        char temp_buf[256];
        int read_len = sp_.readData(temp_buf, sizeof(temp_buf) - 1);
        
        if (read_len > 0) {
            temp_buf[read_len] = '\0';
            read_buffer_ += temp_buf;

            size_t pos;
            // 개행문자(\n) 기준으로 완벽한 한 줄이 들어왔을 때만 파싱 (데이터 쪼개짐 방지)
            while ((pos = read_buffer_.find('\n')) != std::string::npos) {
                std::string line = read_buffer_.substr(0, pos);
                read_buffer_.erase(0, pos + 1);

                // 아두이노에서 보낸 데이터가 "T," 로 시작하는 경우 (Tick 데이터)
                if (line.rfind("T,", 0) == 0) {
                    std::stringstream ss(line);
                    std::string token;
                    std::vector<std::string> tokens;
                    
                    // 콤마(,)를 기준으로 데이터 분리
                    while (std::getline(ss, token, ',')) tokens.push_back(token);

                    if (tokens.size() >= 3) {
                        try {
                            auto tick_msg = std_msgs::msg::Int32MultiArray();
                            tick_msg.data.push_back(std::stoi(tokens[1])); // 왼쪽 틱
                            tick_msg.data.push_back(std::stoi(tokens[2])); // 오른쪽 틱
                            tick_pub_->publish(tick_msg);
                        } catch (...) {
                            // std::stoi 변환 실패 시 무시 (노이즈 방어용 로직)
                            RCLCPP_WARN(this->get_logger(), "손상된 시리얼 데이터 수신됨: %s", line.c_str());
                        }
                    }
                }
            }
        }
    }

    CSerialPort sp_; // CSerialPort 객체
    std::string read_buffer_; // 불완전한 데이터를 모아두는 버퍼
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr tick_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
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
