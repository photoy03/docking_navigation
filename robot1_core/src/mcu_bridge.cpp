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


// =============================================================
// MCU Bridge
// =============================================================

class MCUBridge : public rclcpp::Node
{
public:

    MCUBridge()
    : Node("mcu_bridge_node"),
      read_buffer_(""),
      mcu_state_(-1),
      docked_(false),
      emergency_(false),
      emergency_tx_pending_(false),
      emergency_retry_count_(0),
      emergency_request_time_(std::chrono::steady_clock::now())
    {
        // =====================================================
        // Parameters
        // =====================================================

        this->declare_parameter<std::string>(
            "port_name",
            "/dev/arduino"
        );

        this->declare_parameter<int>(
            "baud_rate",
            115200
        );


        const std::string port_name =
            this->get_parameter(
                "port_name"
            ).as_string();


        const int baud_rate =
            this->get_parameter(
                "baud_rate"
            ).as_int();


        // =====================================================
        // Serial
        // =====================================================

        sp_.init(
            port_name.c_str(),
            baud_rate,
            ParityNone,
            DataBits8,
            StopOne
        );


        if (sp_.open())
        {
            RCLCPP_INFO(
                this->get_logger(),
                "MCU 시리얼 연결 성공: %s",
                port_name.c_str()
            );
        }
        else
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "시리얼 포트 열기 실패: %s",
                port_name.c_str()
            );
        }


        // =====================================================
        // State QoS
        // =====================================================

        auto state_qos =
            rclcpp::QoS(
                rclcpp::KeepLast(1)
            )
            .reliable()
            .transient_local();


        // =====================================================
        // Publishers
        // =====================================================

        tick_pub_ =
            this->create_publisher<
                std_msgs::msg::Int32MultiArray
            >(
                "/robot1/wheel_ticks",
                10
            );


        state_pub_ =
            this->create_publisher<
                std_msgs::msg::Int32
            >(
                "/mcu/state",
                state_qos
            );


        docked_pub_ =
            this->create_publisher<
                std_msgs::msg::Bool
            >(
                "/mcu/docked",
                state_qos
            );


        emergency_pub_ =
            this->create_publisher<
                std_msgs::msg::Bool
            >(
                "/mcu/emergency",
                state_qos
            );


        // =====================================================
        // Subscribers
        // =====================================================

        cmd_sub_ =
            this->create_subscription<
                geometry_msgs::msg::Twist
            >(
                "/cmd_vel",
                10,

                std::bind(
                    &MCUBridge::cmd_callback,
                    this,
                    std::placeholders::_1
                )
            );


        mode_cmd_sub_ =
            this->create_subscription<
                std_msgs::msg::Int32
            >(
                "/mcu/mode_cmd",
                10,

                std::bind(
                    &MCUBridge::mode_cmd_callback,
                    this,
                    std::placeholders::_1
                )
            );


        emergency_cmd_sub_ =
            this->create_subscription<
                std_msgs::msg::Bool
            >(
                "/mcu/emergency_cmd",
                10,

                std::bind(
                    &MCUBridge::emergency_cmd_callback,
                    this,
                    std::placeholders::_1
                )
            );


        // =====================================================
        // Serial RX Timer
        // =====================================================

        read_timer_ =
            this->create_wall_timer(
                10ms,

                std::bind(
                    &MCUBridge::read_serial_callback,
                    this
                )
            );


        // =====================================================
        // Emergency ACK Retry Timer
        //
        // 50 ms마다 pending 상태를 확인하지만
        // 실제 재전송 간격은:
        //
        // 최초 3회 : 200 ms
        // 이후     : 1000 ms
        // =====================================================

        emergency_retry_timer_ =
            this->create_wall_timer(
                50ms,

                std::bind(
                    &MCUBridge::emergency_retry_callback,
                    this
                )
            );


        // =====================================================
        // Start log
        // =====================================================

        RCLCPP_INFO(
            this->get_logger(),
            "========================================"
        );

        RCLCPP_INFO(
            this->get_logger(),
            "MCU Bridge START"
        );

        RCLCPP_INFO(
            this->get_logger(),
            "Protocol: M / V / E / D / S / T / B"
        );

        RCLCPP_INFO(
            this->get_logger(),
            "Emergency recovery E,0 지원"
        );

        RCLCPP_INFO(
            this->get_logger(),
            "Emergency ACK: pending / confirmed 분리"
        );

        RCLCPP_INFO(
            this->get_logger(),
            "Emergency E1 ACK retry 활성화"
        );

        RCLCPP_INFO(
            this->get_logger(),
            "========================================"
        );
    }


    // =========================================================
    // Destructor
    // =========================================================

    ~MCUBridge()
    {
        if (sp_.isOpen())
        {
            sp_.close();
        }
    }


private:

    // =========================================================
    // CSV Split
    // =========================================================

    std::vector<std::string> split_csv(
        const std::string & line
    )
    {
        std::stringstream ss(
            line
        );

        std::string token;

        std::vector<std::string>
            tokens;


        while (
            std::getline(
                ss,
                token,
                ','
            )
        )
        {
            tokens.push_back(
                token
            );
        }


        return tokens;
    }


    // =========================================================
    // cmd_vel
    //
    // STATE2 AUTO에서만 STM으로 전달.
    //
    // Emergency confirmed 또는
    // E1 ACK pending 상태에서는 즉시 차단.
    // =========================================================

    void cmd_callback(
        const geometry_msgs::msg::Twist::SharedPtr msg
    )
    {
        if (!sp_.isOpen())
        {
            return;
        }


        // -----------------------------------------------------
        // Emergency gate
        // -----------------------------------------------------

        if (
            emergency_ ||
            emergency_tx_pending_
        )
        {
            return;
        }


        // -----------------------------------------------------
        // AUTO gate
        // -----------------------------------------------------

        if (mcu_state_ != 2)
        {
            return;
        }


        char buffer[64];


        std::snprintf(
            buffer,
            sizeof(buffer),
            "V,%.6f,%.6f\n",
            msg->linear.x,
            msg->angular.z
        );


        sp_.writeData(
            buffer,
            std::strlen(buffer)
        );
    }


    // =========================================================
    // MCU Mode Command
    //
    // 0 = STOP
    // 1 = ADMITTANCE
    // 2 = AUTO
    //
    // Emergency confirmed/pending에서는
    // M0만 허용.
    // =========================================================

    void mode_cmd_callback(
        const std_msgs::msg::Int32::SharedPtr msg
    )
    {
        if (!sp_.isOpen())
        {
            RCLCPP_WARN(
                this->get_logger(),
                "MCU mode 명령 실패: Serial port closed"
            );

            return;
        }


        const int requested_mode =
            msg->data;


        // -----------------------------------------------------
        // Valid mode
        // -----------------------------------------------------

        if (
            requested_mode < 0 ||
            requested_mode > 2
        )
        {
            RCLCPP_WARN(
                this->get_logger(),
                "잘못된 MCU mode 요청: %d",
                requested_mode
            );

            return;
        }


        // -----------------------------------------------------
        // Emergency gate
        //
        // M0은 안전 STOP이므로 허용.
        // -----------------------------------------------------

        if (
            (
                emergency_ ||
                emergency_tx_pending_
            ) &&
            requested_mode != 0
        )
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "Emergency 상태/pending: M,%d 요청 차단",
                requested_mode
            );

            return;
        }


        char buffer[16];


        std::snprintf(
            buffer,
            sizeof(buffer),
            "M,%d\n",
            requested_mode
        );


        sp_.writeData(
            buffer,
            std::strlen(buffer)
        );


        RCLCPP_INFO(
            this->get_logger(),
            "[MCU TX] M,%d",
            requested_mode
        );
    }


    // =========================================================
    // Emergency Command
    //
    // /mcu/emergency_cmd = true
    //
    // 최초 E1 송신 직후 pending=true.
    //
    // STM ACK를 받기 전부터:
    //
    // - cmd_vel 차단
    // - M1 / M2 차단
    //
    // =========================================================

    void emergency_cmd_callback(
        const std_msgs::msg::Bool::SharedPtr msg
    )
    {
        if (!msg->data)
        {
            return;
        }


        if (!sp_.isOpen())
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "Emergency 명령 실패: Serial port closed"
            );

            return;
        }


        // -----------------------------------------------------
        // Duplicate request 방지
        // -----------------------------------------------------

        if (
            emergency_ ||
            emergency_tx_pending_
        )
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Emergency 요청 무시: 이미 confirmed 또는 pending"
            );

            return;
        }


        // -----------------------------------------------------
        // ACK pending 먼저 설정
        //
        // 이후부터 모든 정상 주행 명령 차단.
        // -----------------------------------------------------

        emergency_tx_pending_ =
            true;


        emergency_retry_count_ =
            0;


        emergency_request_time_ =
            std::chrono::steady_clock::now();


        // -----------------------------------------------------
        // 최초 E1 송신
        // -----------------------------------------------------

        const char * command =
            "E,1\n";


        sp_.writeData(
            command,
            std::strlen(command)
        );


        RCLCPP_ERROR(
            this->get_logger(),
            "[MCU TX] E,1 - Emergency 요청 (ACK 대기)"
        );
    }


    // =========================================================
    // Emergency ACK Retry
    //
    // 최초 E1 송신 후 STM의 E,1,<reason> ACK가 없으면:
    //
    // retry #1 : +200 ms
    // retry #2 : +200 ms
    // retry #3 : +200 ms
    //
    // 이후:
    //
    // 1초 간격으로 계속 E1 재전송.
    //
    // ACK 없이 emergency pending을 자동 해제하지 않는다.
    // =========================================================

    void emergency_retry_callback()
    {
        // -----------------------------------------------------
        // Pending 아님
        // -----------------------------------------------------

        if (!emergency_tx_pending_)
        {
            return;
        }


        // -----------------------------------------------------
        // 이미 ACK 확인
        // -----------------------------------------------------

        if (emergency_)
        {
            return;
        }


        // -----------------------------------------------------
        // Serial closed
        //
        // pending은 그대로 유지.
        // 정상 주행 명령은 계속 차단됨.
        // -----------------------------------------------------

        if (!sp_.isOpen())
        {
            RCLCPP_ERROR_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                5000,
                "Emergency pending: Serial port closed"
            );

            return;
        }


        const auto now =
            std::chrono::steady_clock::now();


        const auto elapsed_ms =
            std::chrono::duration_cast<
                std::chrono::milliseconds
            >(
                now -
                emergency_request_time_
            ).count();


        // -----------------------------------------------------
        // 처음 3회 빠른 retry
        // 이후 1초 간격 지속 retry
        // -----------------------------------------------------

        constexpr int
            FAST_RETRY_LIMIT =
                3;


        constexpr long
            FAST_RETRY_INTERVAL_MS =
                200;


        constexpr long
            CONTINUOUS_RETRY_INTERVAL_MS =
                1000;


        const long retry_interval_ms =
            (
                emergency_retry_count_ <
                FAST_RETRY_LIMIT
            )
            ?
            FAST_RETRY_INTERVAL_MS
            :
            CONTINUOUS_RETRY_INTERVAL_MS;


        if (
            elapsed_ms <
            retry_interval_ms
        )
        {
            return;
        }


        // -----------------------------------------------------
        // E1 재전송
        // -----------------------------------------------------

        const char * command =
            "E,1\n";


        sp_.writeData(
            command,
            std::strlen(command)
        );


        ++emergency_retry_count_;


        emergency_request_time_ =
            now;


        // -----------------------------------------------------
        // Fast retry log
        // -----------------------------------------------------

        if (
            emergency_retry_count_ <=
            FAST_RETRY_LIMIT
        )
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "[MCU TX] E,1 retry (%d/%d)",
                emergency_retry_count_,
                FAST_RETRY_LIMIT
            );
        }

        // -----------------------------------------------------
        // Continuous retry log
        // -----------------------------------------------------

        else
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "[MCU TX] Emergency ACK 없음 - E,1 계속 재전송 (#%d)",
                emergency_retry_count_
            );
        }
    }


    // =========================================================
    // Serial RX
    // =========================================================

    void read_serial_callback()
    {
        if (!sp_.isOpen())
        {
            return;
        }


        char temp_buf[256];


        const int read_len =
            sp_.readData(
                temp_buf,
                sizeof(temp_buf) - 1
            );


        if (read_len <= 0)
        {
            return;
        }


        temp_buf[read_len] =
            '\0';


        read_buffer_ +=
            temp_buf;


        size_t pos;


        while (
            (
                pos =
                    read_buffer_.find('\n')
            ) !=
            std::string::npos
        )
        {
            std::string line =
                read_buffer_.substr(
                    0,
                    pos
                );


            read_buffer_.erase(
                0,
                pos + 1
            );


            if (
                !line.empty() &&
                line.back() == '\r'
            )
            {
                line.pop_back();
            }


            if (line.empty())
            {
                continue;
            }


            parse_serial_line(
                line
            );
        }
    }


    // =========================================================
    // Parse MCU packet
    // =========================================================

    void parse_serial_line(
        const std::string & line
    )
    {
        const auto tokens =
            split_csv(
                line
            );


        if (tokens.empty())
        {
            return;
        }


        // =====================================================
        // B,READY
        // B,TARE_OK
        // =====================================================

        if (tokens[0] == "B")
        {
            if (tokens.size() < 2)
            {
                return;
            }


            const std::string boot_msg =
                tokens[1];


            RCLCPP_INFO(
                this->get_logger(),
                "[MCU BOOT] %s",
                boot_msg.c_str()
            );


            // -------------------------------------------------
            // STM reboot / READY
            // -------------------------------------------------

            if (boot_msg == "READY")
            {
                mcu_state_ =
                    0;


                docked_ =
                    false;


                emergency_ =
                    false;


                emergency_tx_pending_ =
                    false;


                emergency_retry_count_ =
                    0;


                // ---------------------------------------------
                // State
                // ---------------------------------------------

                std_msgs::msg::Int32
                    state_msg;


                state_msg.data =
                    0;


                state_pub_->publish(
                    state_msg
                );


                // ---------------------------------------------
                // Dock
                // ---------------------------------------------

                std_msgs::msg::Bool
                    dock_msg;


                dock_msg.data =
                    false;


                docked_pub_->publish(
                    dock_msg
                );


                // ---------------------------------------------
                // Emergency
                // ---------------------------------------------

                std_msgs::msg::Bool
                    emergency_msg;


                emergency_msg.data =
                    false;


                emergency_pub_->publish(
                    emergency_msg
                );
            }


            return;
        }


        // =====================================================
        // D,1
        //
        // First docking complete
        // =====================================================

        if (tokens[0] == "D")
        {
            if (tokens.size() < 2)
            {
                return;
            }


            try
            {
                const int value =
                    std::stoi(
                        tokens[1]
                    );


                docked_ =
                    (value == 1);


                std_msgs::msg::Bool
                    msg;


                msg.data =
                    docked_;


                docked_pub_->publish(
                    msg
                );


                RCLCPP_INFO(
                    this->get_logger(),
                    "[MCU DOCK] docked=%s",
                    docked_
                        ? "true"
                        : "false"
                );
            }
            catch (...)
            {
                RCLCPP_WARN(
                    this->get_logger(),
                    "잘못된 Dock 패킷: %s",
                    line.c_str()
                );
            }


            return;
        }


        // =====================================================
        // S,0 STOP / EMERGENCY
        // S,1 ADMITTANCE
        // S,2 AUTO
        // =====================================================

        if (tokens[0] == "S")
        {
            if (tokens.size() < 2)
            {
                return;
            }


            try
            {
                const int state =
                    std::stoi(
                        tokens[1]
                    );


                if (
                    state < 0 ||
                    state > 2
                )
                {
                    RCLCPP_WARN(
                        this->get_logger(),
                        "알 수 없는 MCU state: %d",
                        state
                    );

                    return;
                }


                mcu_state_ =
                    state;


                std_msgs::msg::Int32
                    msg;


                msg.data =
                    mcu_state_;


                state_pub_->publish(
                    msg
                );


                const char * state_name =
                    "UNKNOWN";


                if (state == 0)
                {
                    state_name =
                        "STOP";
                }
                else if (state == 1)
                {
                    state_name =
                        "ADMITTANCE";
                }
                else if (state == 2)
                {
                    state_name =
                        "AUTO";
                }


                RCLCPP_INFO(
                    this->get_logger(),
                    "[MCU STATE] %d (%s)",
                    state,
                    state_name
                );
            }
            catch (...)
            {
                RCLCPP_WARN(
                    this->get_logger(),
                    "잘못된 State 패킷: %s",
                    line.c_str()
                );
            }


            return;
        }


        // =====================================================
        // T,left_total,right_total
        // =====================================================

        if (tokens[0] == "T")
        {
            if (tokens.size() < 3)
            {
                return;
            }


            try
            {
                const int left_tick =
                    std::stoi(
                        tokens[1]
                    );


                const int right_tick =
                    std::stoi(
                        tokens[2]
                    );


                auto tick_msg =
                    std_msgs::msg::Int32MultiArray();


                tick_msg.data.push_back(
                    left_tick
                );


                tick_msg.data.push_back(
                    right_tick
                );


                tick_pub_->publish(
                    tick_msg
                );
            }
            catch (...)
            {
                RCLCPP_WARN(
                    this->get_logger(),
                    "손상된 Tick 패킷: %s",
                    line.c_str()
                );
            }


            return;
        }


        // =====================================================
        // E,1,<reason>
        // E,0
        // =====================================================

        if (tokens[0] == "E")
        {
            if (tokens.size() < 2)
            {
                return;
            }


            try
            {
                const int value =
                    std::stoi(
                        tokens[1]
                    );


                // =============================================
                // Emergency SET ACK
                // =============================================

                if (value == 1)
                {
                    emergency_ =
                        true;


                    emergency_tx_pending_ =
                        false;


                    emergency_retry_count_ =
                        0;


                    std_msgs::msg::Bool
                        msg;


                    msg.data =
                        true;


                    emergency_pub_->publish(
                        msg
                    );


                    std::string reason =
                        "UNKNOWN";


                    if (tokens.size() >= 3)
                    {
                        reason =
                            tokens[2];
                    }


                    RCLCPP_ERROR(
                        this->get_logger(),
                        "[MCU EMERGENCY SET] reason=%s",
                        reason.c_str()
                    );


                    return;
                }


                // =============================================
                // Emergency CLEAR
                // =============================================

                if (value == 0)
                {
                    emergency_ =
                        false;


                    emergency_tx_pending_ =
                        false;


                    emergency_retry_count_ =
                        0;


                    std_msgs::msg::Bool
                        msg;


                    msg.data =
                        false;


                    emergency_pub_->publish(
                        msg
                    );


                    RCLCPP_INFO(
                        this->get_logger(),
                        "[MCU EMERGENCY CLEAR] E,0"
                    );


                    return;
                }


                RCLCPP_WARN(
                    this->get_logger(),
                    "알 수 없는 Emergency value: %d",
                    value
                );
            }
            catch (...)
            {
                RCLCPP_WARN(
                    this->get_logger(),
                    "잘못된 Emergency 패킷: %s",
                    line.c_str()
                );
            }


            return;
        }


        // =====================================================
        // PONG
        // =====================================================

        if (line == "PONG")
        {
            RCLCPP_INFO(
                this->get_logger(),
                "[MCU] PONG"
            );

            return;
        }


        // =====================================================
        // Temporary STM debug telemetry
        //
        // L / A packet은 ROS 상태로 처리하지 않음
        // =====================================================

        if (
            tokens[0] == "L" ||
            tokens[0] == "A"
        )
        {
            return;
        }


        RCLCPP_DEBUG(
            this->get_logger(),
            "알 수 없는 MCU packet: %s",
            line.c_str()
        );
    }


    // =========================================================
    // Serial
    // =========================================================

    CSerialPort
        sp_;


    std::string
        read_buffer_;


    // =========================================================
    // MCU State
    // =========================================================

    int
        mcu_state_;


    bool
        docked_;


    // =========================================================
    // Emergency State
    // =========================================================

    // STM에서 E,1 ACK를 받아 확인된 Emergency
    bool
        emergency_;


    // Jetson이 E,1을 보냈지만
    // 아직 STM ACK를 받지 못한 상태
    bool
        emergency_tx_pending_;


    // E1 재전송 횟수
    int
        emergency_retry_count_;


    // 마지막 E1 송신 시각
    std::chrono::steady_clock::time_point
        emergency_request_time_;


    // =========================================================
    // Publishers
    // =========================================================

    rclcpp::Publisher<
        std_msgs::msg::Int32MultiArray
    >::SharedPtr
        tick_pub_;


    rclcpp::Publisher<
        std_msgs::msg::Int32
    >::SharedPtr
        state_pub_;


    rclcpp::Publisher<
        std_msgs::msg::Bool
    >::SharedPtr
        docked_pub_;


    rclcpp::Publisher<
        std_msgs::msg::Bool
    >::SharedPtr
        emergency_pub_;


    // =========================================================
    // Subscribers
    // =========================================================

    rclcpp::Subscription<
        geometry_msgs::msg::Twist
    >::SharedPtr
        cmd_sub_;


    rclcpp::Subscription<
        std_msgs::msg::Int32
    >::SharedPtr
        mode_cmd_sub_;


    rclcpp::Subscription<
        std_msgs::msg::Bool
    >::SharedPtr
        emergency_cmd_sub_;


    // =========================================================
    // Timers
    // =========================================================

    rclcpp::TimerBase::SharedPtr
        read_timer_;


    rclcpp::TimerBase::SharedPtr
        emergency_retry_timer_;
};


// =============================================================
// MAIN
// =============================================================

int main(
    int argc,
    char * argv[]
)
{
    rclcpp::init(
        argc,
        argv
    );


    auto node =
        std::make_shared<
            MCUBridge
        >();


    rclcpp::spin(
        node
    );


    rclcpp::shutdown();


    return 0;
}
