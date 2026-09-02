#include <chrono>
#include <cstdint>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "nav2_msgs/srv/manage_lifecycle_nodes.hpp"

using namespace std::chrono_literals;


class LocalizationStartupGate : public rclcpp::Node
{
public:
    LocalizationStartupGate()
    : Node("localization_startup_gate"),
      docked_(false),
      request_in_flight_(false),
      startup_done_(false)
    {
        // /mcu/docked publisher가 transient_local이므로
        // gate도 동일한 durability를 사용한다.
        auto dock_qos =
            rclcpp::QoS(rclcpp::KeepLast(1))
                .reliable()
                .transient_local();

        dock_sub_ =
            this->create_subscription<std_msgs::msg::Bool>(
                "/mcu/docked",
                dock_qos,
                std::bind(
                    &LocalizationStartupGate::dock_callback,
                    this,
                    std::placeholders::_1
                )
            );


        lifecycle_client_ =
            this->create_client<
                nav2_msgs::srv::ManageLifecycleNodes
            >(
                "/lifecycle_manager_localization/manage_nodes"
            );


        // D,1이 먼저 들어와도 lifecycle manager 서비스가
        // 아직 생성되지 않았을 수 있으므로 주기적으로 확인한다.
        timer_ =
            this->create_wall_timer(
                500ms,
                std::bind(
                    &LocalizationStartupGate::timer_callback,
                    this
                )
            );


        RCLCPP_INFO(
            this->get_logger(),
            "Localization Startup Gate 시작"
        );

        RCLCPP_INFO(
            this->get_logger(),
            "도킹 완료 신호 /mcu/docked=true 대기 중"
        );
    }


private:

    void dock_callback(
        const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (!msg->data)
        {
            docked_ = false;

            RCLCPP_INFO(
                this->get_logger(),
                "MCU docked=false"
            );

            return;
        }


        if (!docked_)
        {
            RCLCPP_INFO(
                this->get_logger(),
                "MCU 도킹 완료 감지"
            );
        }

        docked_ = true;
    }


    void timer_callback()
    {
        // 아직 도킹 전
        if (!docked_) {
            return;
        }

        // 이미 Localization startup 완료
        if (startup_done_) {
            return;
        }

        // 이전 요청 응답 대기 중
        if (request_in_flight_) {
            return;
        }


        // Lifecycle manager가 아직 실행되지 않았다면 대기
        if (!lifecycle_client_->service_is_ready())
        {
            RCLCPP_INFO_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "Localization lifecycle service 대기 중..."
            );

            return;
        }


        auto request =
            std::make_shared<
                nav2_msgs::srv::ManageLifecycleNodes::Request
            >();


        // ManageLifecycleNodes:
        // STARTUP = 0
        request->command = 0;

        request_in_flight_ = true;


        RCLCPP_INFO(
            this->get_logger(),
            "Localization Lifecycle STARTUP 요청"
        );


        lifecycle_client_->async_send_request(
            request,

            [this](
                rclcpp::Client<
                    nav2_msgs::srv::ManageLifecycleNodes
                >::SharedFuture future)
            {
                request_in_flight_ = false;

                try
                {
                    auto response =
                        future.get();

                    if (response->success)
                    {
                        startup_done_ = true;

                        RCLCPP_INFO(
                            this->get_logger(),
                            "Localization Lifecycle STARTUP 성공"
                        );
                    }
                    else
                    {
                        RCLCPP_ERROR(
                            this->get_logger(),
                            "Localization Lifecycle STARTUP 실패 - 재시도 예정"
                        );
                    }
                }
                catch (const std::exception & e)
                {
                    RCLCPP_ERROR(
                        this->get_logger(),
                        "Lifecycle service 예외: %s",
                        e.what()
                    );
                }
            }
        );
    }


    bool docked_;
    bool request_in_flight_;
    bool startup_done_;


    rclcpp::Subscription<
        std_msgs::msg::Bool
    >::SharedPtr dock_sub_;


    rclcpp::Client<
        nav2_msgs::srv::ManageLifecycleNodes
    >::SharedPtr lifecycle_client_;


    rclcpp::TimerBase::SharedPtr timer_;
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node =
        std::make_shared<LocalizationStartupGate>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}