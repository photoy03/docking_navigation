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
        // TF
        // =====================================================

        tf_buffer_ =
            std::make_unique<tf2_ros::Buffer>(
                this->get_clock()
            );

        tf_listener_ =
            std::make_shared<tf2_ros::TransformListener>(
                *tf_buffer_
            );


        // =====================================================
        // /scan_filtered
        // =====================================================

        scan_sub_ =
            this->create_subscription<sensor_msgs::msg::LaserScan>(
                "/scan_filtered",
                rclcpp::SensorDataQoS(),
                [this](sensor_msgs::msg::LaserScan::SharedPtr)
                {
                    scan_received_ = true;
                }
            );


        // =====================================================
        // /robot1/odom
        // =====================================================

        odom_sub_ =
            this->create_subscription<nav_msgs::msg::Odometry>(
                "/robot1/odom",
                rclcpp::SensorDataQoS(),
                [this](nav_msgs::msg::Odometry::SharedPtr)
                {
                    odom_received_ = true;
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
                    geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr)
                {
                    amcl_pose_received_ = true;
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


        RCLCPP_INFO(
            this->get_logger(),
            "Localization Ready Gate 시작"
        );

        RCLCPP_INFO(
            this->get_logger(),
            "scan / odom / amcl_pose / map->robot1_odom 대기 중"
        );
    }


private:

    void check_ready()
    {
        if (navigation_started_) {
            return;
        }

        if (request_in_flight_) {
            return;
        }


        // =====================================================
        // Sensor / localization topics
        // =====================================================

        if (!scan_received_) {
            return;
        }

        if (!odom_received_) {
            return;
        }

        if (!amcl_pose_received_) {
            return;
        }


        // =====================================================
        // map -> robot1_odom TF
        // =====================================================

        bool tf_ready = false;

        try
        {
            tf_ready =
                tf_buffer_->canTransform(
                    "map",
                    "robot1_odom",
                    tf2::TimePointZero
                );
        }
        catch (const tf2::TransformException &)
        {
            tf_ready = false;
        }


        if (!tf_ready) {
            return;
        }


        // =====================================================
        // Navigation lifecycle service
        // =====================================================

        if (!navigation_lifecycle_client_->service_is_ready())
        {
            RCLCPP_INFO_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "Navigation lifecycle service 대기 중..."
            );

            return;
        }


        RCLCPP_INFO(
            this->get_logger(),
            "========================================"
        );

        RCLCPP_INFO(
            this->get_logger(),
            "Localization Ready"
        );

        RCLCPP_INFO(
            this->get_logger(),
            "/scan_filtered        : OK"
        );

        RCLCPP_INFO(
            this->get_logger(),
            "/robot1/odom          : OK"
        );

        RCLCPP_INFO(
            this->get_logger(),
            "/amcl_pose            : OK"
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


    void start_navigation()
    {
        using ManageLifecycleNodes =
            nav2_msgs::srv::ManageLifecycleNodes;


        auto request =
            std::make_shared<
                ManageLifecycleNodes::Request
            >();


        // STARTUP = 0
        request->command = 0;


        request_in_flight_ = true;


        RCLCPP_INFO(
            this->get_logger(),
            "Navigation Lifecycle STARTUP 요청"
        );


        navigation_lifecycle_client_->async_send_request(

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
                        navigation_started_ = true;

                        RCLCPP_INFO(
                            this->get_logger(),
                            "Navigation Lifecycle STARTUP 성공"
                        );
                    }
                    else
                    {
                        RCLCPP_ERROR(
                            this->get_logger(),
                            "Navigation Lifecycle STARTUP 실패 - 재시도"
                        );
                    }
                }
                catch (const std::exception & e)
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


    bool scan_received_;
    bool odom_received_;
    bool amcl_pose_received_;

    bool request_in_flight_;
    bool navigation_started_;


    rclcpp::Subscription<
        sensor_msgs::msg::LaserScan
    >::SharedPtr scan_sub_;


    rclcpp::Subscription<
        nav_msgs::msg::Odometry
    >::SharedPtr odom_sub_;


    rclcpp::Subscription<
        geometry_msgs::msg::PoseWithCovarianceStamped
    >::SharedPtr amcl_pose_sub_;


    rclcpp::Client<
        nav2_msgs::srv::ManageLifecycleNodes
    >::SharedPtr navigation_lifecycle_client_;


    std::unique_ptr<
        tf2_ros::Buffer
    > tf_buffer_;


    std::shared_ptr<
        tf2_ros::TransformListener
    > tf_listener_;


    rclcpp::TimerBase::SharedPtr
        check_timer_;
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node =
        std::make_shared<LocalizationReadyGate>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}