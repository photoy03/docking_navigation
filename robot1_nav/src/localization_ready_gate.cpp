#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

#include "nav2_msgs/srv/manage_lifecycle_nodes.hpp"

#include "tf2/time.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"


using namespace std::chrono_literals;


// =============================================================
// Localization Ready Gate
//
// Navigation lifecycle은 다음 조건이 모두 만족될 때만 STARTUP:
//
// 1. /scan_filtered 최근 수신
// 2. /robot1/odom 최근 수신
// 3. /amcl_pose 최근 수신
// 4. map -> robot1_odom TF 존재
//
// 단순히 "한 번 받은 적 있음"이 아니라
// 실제 최근 데이터가 살아있는지 확인한다.
// =============================================================

class LocalizationReadyGate : public rclcpp::Node
{
public:

    LocalizationReadyGate()
    : Node("localization_ready_gate"),
      scan_received_(false),
      odom_received_(false),
      amcl_pose_received_(false),
      request_in_flight_(false),
      navigation_started_(false)
    {
        // =====================================================
        // Parameters
        // =====================================================

        // RPLIDAR A2M12 약 10 Hz 기준.
        // 0.5초 동안 scan이 없다면 startup 허용하지 않음.
        this->declare_parameter<double>(
            "scan_timeout_sec",
            0.5
        );


        // wheel tick / odom은 훨씬 빠르므로
        // 0.5초면 충분히 여유 있는 값.
        this->declare_parameter<double>(
            "odom_timeout_sec",
            0.5
        );


        // AMCL pose는 scan/odom보다 느릴 수 있으므로
        // 너무 빡빡하게 잡지 않음.
        this->declare_parameter<double>(
            "amcl_pose_timeout_sec",
            2.0
        );


        scan_timeout_sec_ =
            this->get_parameter(
                "scan_timeout_sec"
            ).as_double();


        odom_timeout_sec_ =
            this->get_parameter(
                "odom_timeout_sec"
            ).as_double();


        amcl_pose_timeout_sec_ =
            this->get_parameter(
                "amcl_pose_timeout_sec"
            ).as_double();


        // =====================================================
        // TF
        // =====================================================

        tf_buffer_ =
            std::make_unique<
                tf2_ros::Buffer
            >(
                this->get_clock()
            );


        tf_listener_ =
            std::make_shared<
                tf2_ros::TransformListener
            >(
                *tf_buffer_
            );


        // =====================================================
        // /scan_filtered
        // =====================================================

        scan_sub_ =
            this->create_subscription<
                sensor_msgs::msg::LaserScan
            >(
                "/scan_filtered",
                rclcpp::SensorDataQoS(),

                [this](
                    sensor_msgs::msg::LaserScan::SharedPtr
                )
                {
                    scan_received_ =
                        true;


                    last_scan_time_ =
                        std::chrono::steady_clock::now();
                }
            );


        // =====================================================
        // /robot1/odom
        // =====================================================

        odom_sub_ =
            this->create_subscription<
                nav_msgs::msg::Odometry
            >(
                "/robot1/odom",
                rclcpp::SensorDataQoS(),

                [this](
                    nav_msgs::msg::Odometry::SharedPtr
                )
                {
                    odom_received_ =
                        true;


                    last_odom_time_ =
                        std::chrono::steady_clock::now();
                }
            );


        // =====================================================
        // /amcl_pose
        // =====================================================

        amcl_pose_sub_ =
            this->create_subscription<
                geometry_msgs::msg::PoseWithCovarianceStamped
            >(
                "/amcl_pose",
                10,

                [this](
                    geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr
                )
                {
                    amcl_pose_received_ =
                        true;


                    last_amcl_pose_time_ =
                        std::chrono::steady_clock::now();
                }
            );


        // =====================================================
        // Navigation lifecycle manager
        // =====================================================

        navigation_lifecycle_client_ =
            this->create_client<
                nav2_msgs::srv::ManageLifecycleNodes
            >(
                "/lifecycle_manager_navigation/manage_nodes"
            );


        // =====================================================
        // Ready check
        // =====================================================

        check_timer_ =
            this->create_wall_timer(
                500ms,

                std::bind(
                    &LocalizationReadyGate::check_ready,
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
            "Localization Ready Gate START"
        );

        RCLCPP_INFO(
            this->get_logger(),
            "Freshness check 활성화"
        );

        RCLCPP_INFO(
            this->get_logger(),
            "scan timeout = %.2fs",
            scan_timeout_sec_
        );

        RCLCPP_INFO(
            this->get_logger(),
            "odom timeout = %.2fs",
            odom_timeout_sec_
        );

        RCLCPP_INFO(
            this->get_logger(),
            "AMCL pose timeout = %.2fs",
            amcl_pose_timeout_sec_
        );

        RCLCPP_INFO(
            this->get_logger(),
            "scan / odom / amcl_pose / map->robot1_odom 대기 중"
        );

        RCLCPP_INFO(
            this->get_logger(),
            "========================================"
        );
    }


private:

    // =========================================================
    // Ready Check
    // =========================================================

    void check_ready()
    {
        // -----------------------------------------------------
        // 이미 Navigation STARTUP 완료
        // -----------------------------------------------------

        if (navigation_started_)
        {
            return;
        }


        // -----------------------------------------------------
        // Lifecycle request 처리 중
        // -----------------------------------------------------

        if (request_in_flight_)
        {
            return;
        }


        const auto now =
            std::chrono::steady_clock::now();


        // =====================================================
        // Scan
        // =====================================================

        if (!scan_received_)
        {
            RCLCPP_INFO_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "Navigation 대기: /scan_filtered 미수신"
            );

            return;
        }


        const double scan_age =
            std::chrono::duration<double>(
                now -
                last_scan_time_
            ).count();


        if (
            scan_age >
            scan_timeout_sec_
        )
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "Navigation 대기: /scan_filtered stale (age=%.3fs)",
                scan_age
            );

            return;
        }


        // =====================================================
        // Odom
        // =====================================================

        if (!odom_received_)
        {
            RCLCPP_INFO_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "Navigation 대기: /robot1/odom 미수신"
            );

            return;
        }


        const double odom_age =
            std::chrono::duration<double>(
                now -
                last_odom_time_
            ).count();


        if (
            odom_age >
            odom_timeout_sec_
        )
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "Navigation 대기: /robot1/odom stale (age=%.3fs)",
                odom_age
            );

            return;
        }


        // =====================================================
        // AMCL Pose
        // =====================================================

        if (!amcl_pose_received_)
        {
            RCLCPP_INFO_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "Navigation 대기: /amcl_pose 미수신"
            );

            return;
        }


        const double amcl_pose_age =
            std::chrono::duration<double>(
                now -
                last_amcl_pose_time_
            ).count();


        if (
            amcl_pose_age >
            amcl_pose_timeout_sec_
        )
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "Navigation 대기: /amcl_pose stale (age=%.3fs)",
                amcl_pose_age
            );

            return;
        }


        // =====================================================
        // map -> robot1_odom TF
        //
        // AMCL pose 자체의 freshness를 위에서 확인했으므로
        // 여기서는 실제 TF chain 존재 여부를 검사.
        // =====================================================

        bool tf_ready =
            false;


        try
        {
            tf_ready =
                tf_buffer_->canTransform(
                    "map",
                    "robot1_odom",
                    tf2::TimePointZero
                );
        }
        catch (
            const tf2::TransformException &
        )
        {
            tf_ready =
                false;
        }


        if (!tf_ready)
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "Navigation 대기: map -> robot1_odom TF 없음"
            );

            return;
        }


        // =====================================================
        // Navigation Lifecycle Service
        // =====================================================

        if (
            !navigation_lifecycle_client_->
                service_is_ready()
        )
        {
            RCLCPP_INFO_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "Navigation lifecycle service 대기 중..."
            );

            return;
        }


        // =====================================================
        // READY
        // =====================================================

        RCLCPP_INFO(
            this->get_logger(),
            "========================================"
        );

        RCLCPP_INFO(
            this->get_logger(),
            "Localization READY"
        );

        RCLCPP_INFO(
            this->get_logger(),
            "/scan_filtered : OK (age=%.3fs)",
            scan_age
        );

        RCLCPP_INFO(
            this->get_logger(),
            "/robot1/odom : OK (age=%.3fs)",
            odom_age
        );

        RCLCPP_INFO(
            this->get_logger(),
            "/amcl_pose : OK (age=%.3fs)",
            amcl_pose_age
        );

        RCLCPP_INFO(
            this->get_logger(),
            "map -> robot1_odom TF : OK"
        );

        RCLCPP_INFO(
            this->get_logger(),
            "========================================"
        );


        start_navigation();
    }


    // =========================================================
    // Navigation STARTUP
    // =========================================================

    void start_navigation()
    {
        using ManageLifecycleNodes =
            nav2_msgs::srv::ManageLifecycleNodes;


        auto request =
            std::make_shared<
                ManageLifecycleNodes::Request
            >();


        // STARTUP = 0
        request->command =
            0;


        request_in_flight_ =
            true;


        RCLCPP_INFO(
            this->get_logger(),
            "Navigation Lifecycle STARTUP 요청"
        );


        navigation_lifecycle_client_->
            async_send_request(
                request,

                [this](
                    rclcpp::Client<
                        nav2_msgs::srv::ManageLifecycleNodes
                    >::SharedFuture future
                )
                {
                    request_in_flight_ =
                        false;


                    try
                    {
                        auto response =
                            future.get();


                        if (response->success)
                        {
                            navigation_started_ =
                                true;


                            RCLCPP_INFO(
                                this->get_logger(),
                                "========================================"
                            );

                            RCLCPP_INFO(
                                this->get_logger(),
                                "Navigation Lifecycle STARTUP 성공"
                            );

                            RCLCPP_INFO(
                                this->get_logger(),
                                "Localization Ready Gate 완료"
                            );

                            RCLCPP_INFO(
                                this->get_logger(),
                                "========================================"
                            );
                        }
                        else
                        {
                            RCLCPP_ERROR(
                                this->get_logger(),
                                "Navigation Lifecycle STARTUP 실패 - 조건 재확인 후 재시도"
                            );
                        }
                    }
                    catch (
                        const std::exception & e
                    )
                    {
                        RCLCPP_ERROR(
                            this->get_logger(),
                            "Navigation Lifecycle service 오류: %s",
                            e.what()
                        );
                    }
                }
            );
    }


    // =========================================================
    // Parameters
    // =========================================================

    double
        scan_timeout_sec_;


    double
        odom_timeout_sec_;


    double
        amcl_pose_timeout_sec_;


    // =========================================================
    // Receive State
    // =========================================================

    bool
        scan_received_;


    bool
        odom_received_;


    bool
        amcl_pose_received_;


    std::chrono::steady_clock::time_point
        last_scan_time_;


    std::chrono::steady_clock::time_point
        last_odom_time_;


    std::chrono::steady_clock::time_point
        last_amcl_pose_time_;


    // =========================================================
    // Lifecycle State
    // =========================================================

    bool
        request_in_flight_;


    bool
        navigation_started_;


    // =========================================================
    // Subscribers
    // =========================================================

    rclcpp::Subscription<
        sensor_msgs::msg::LaserScan
    >::SharedPtr
        scan_sub_;


    rclcpp::Subscription<
        nav_msgs::msg::Odometry
    >::SharedPtr
        odom_sub_;


    rclcpp::Subscription<
        geometry_msgs::msg::PoseWithCovarianceStamped
    >::SharedPtr
        amcl_pose_sub_;


    // =========================================================
    // Lifecycle Client
    // =========================================================

    rclcpp::Client<
        nav2_msgs::srv::ManageLifecycleNodes
    >::SharedPtr
        navigation_lifecycle_client_;


    // =========================================================
    // TF
    // =========================================================

    std::unique_ptr<
        tf2_ros::Buffer
    >
        tf_buffer_;


    std::shared_ptr<
        tf2_ros::TransformListener
    >
        tf_listener_;


    // =========================================================
    // Timer
    // =========================================================

    rclcpp::TimerBase::SharedPtr
        check_timer_;
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
            LocalizationReadyGate
        >();


    rclcpp::spin(
        node
    );


    rclcpp::shutdown();


    return 0;
}
