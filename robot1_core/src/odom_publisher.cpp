#include <chrono>
#include <memory>
#include <cmath>
#include <cstdint>

#include "rclcpp/rclcpp.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"


using namespace std::chrono_literals;


class OdomPublisher : public rclcpp::Node
{
public:

    OdomPublisher()
    : Node("robot1_odom_node"),
      x_(0.0),
      y_(0.0),
      theta_(0.0),
      imu_yaw_(0.0),
      imu_angular_z_(0.0),
      imu_received_(false),
      imu_fresh_last_cycle_(false),
      first_tick_received_(false),
      prev_left_tick_(0),
      prev_right_tick_(0)
    {
        // =====================================================
        // Hardware parameters
        // =====================================================

        this->declare_parameter<double>(
            "cpr",
            3588.0
        );

        this->declare_parameter<double>(
            "wheel_diameter",
            0.216
        );

        this->declare_parameter<double>(
            "track_width",
            1.2405
        );


        // =====================================================
        // IMU fusion
        // =====================================================

        this->declare_parameter<double>(
            "imu_trust_factor",
            0.1
        );

        this->declare_parameter<double>(
            "enc_v_theta_weight",
            0.001
        );


        // IMU가 이 시간 이상 갱신되지 않으면
        // encoder-only odometry로 fallback
        this->declare_parameter<double>(
            "imu_timeout_sec",
            0.30
        );


        // =====================================================
        // Encoder safety
        // =====================================================

        // 정상 tick은 약 50 Hz.
        // 0.25초 이상 gap이면 해당 구간을 적분하지 않고
        // 현재 tick을 새로운 기준값으로 사용.
        this->declare_parameter<double>(
            "max_tick_dt_sec",
            0.25
        );


        // 현재 명령 한계에서 wheel 최대 약 1.53 m/s.
        // 약간의 여유를 두어 2.0 m/s 사용.
        this->declare_parameter<double>(
            "max_wheel_speed_mps",
            2.0
        );


        // timestamp jitter / encoder quantization 여유
        this->declare_parameter<double>(
            "tick_jump_margin_m",
            0.05
        );


        // =====================================================
        // Get parameters
        // =====================================================

        cpr_ =
            this->get_parameter(
                "cpr"
            ).as_double();

        wheel_diameter_ =
            this->get_parameter(
                "wheel_diameter"
            ).as_double();

        track_width_ =
            this->get_parameter(
                "track_width"
            ).as_double();

        imu_trust_factor_ =
            this->get_parameter(
                "imu_trust_factor"
            ).as_double();

        enc_v_theta_weight_ =
            this->get_parameter(
                "enc_v_theta_weight"
            ).as_double();

        imu_timeout_sec_ =
            this->get_parameter(
                "imu_timeout_sec"
            ).as_double();

        max_tick_dt_sec_ =
            this->get_parameter(
                "max_tick_dt_sec"
            ).as_double();

        max_wheel_speed_mps_ =
            this->get_parameter(
                "max_wheel_speed_mps"
            ).as_double();

        tick_jump_margin_m_ =
            this->get_parameter(
                "tick_jump_margin_m"
            ).as_double();


        distance_per_tick_ =
            (
                M_PI *
                wheel_diameter_
            ) /
            cpr_;


        // =====================================================
        // Publishers
        // =====================================================

        odom_pub_ =
            this->create_publisher<
                nav_msgs::msg::Odometry
            >(
                "/robot1/odom",
                10
            );


        tf_broadcaster_ =
            std::make_unique<
                tf2_ros::TransformBroadcaster
            >(
                *this
            );


        // =====================================================
        // Subscribers
        // =====================================================

        tick_sub_ =
            this->create_subscription<
                std_msgs::msg::Int32MultiArray
            >(
                "/robot1/wheel_ticks",
                10,

                std::bind(
                    &OdomPublisher::tick_callback,
                    this,
                    std::placeholders::_1
                )
            );


        // navigation_launch.py에서
        //
        // /robot1/imu/data
        //      ↓ remap
        // /imu/data
        //
        // 로 연결됨.
        imu_sub_ =
            this->create_subscription<
                sensor_msgs::msg::Imu
            >(
                "/robot1/imu/data",
                10,

                std::bind(
                    &OdomPublisher::imu_callback,
                    this,
                    std::placeholders::_1
                )
            );


        last_time_ =
            this->now();

        last_imu_time_ =
            this->now();


        RCLCPP_INFO(
            this->get_logger(),
            "========================================"
        );

        RCLCPP_INFO(
            this->get_logger(),
            "Robot1 Odom Publisher START"
        );

        RCLCPP_INFO(
            this->get_logger(),
            "CPR=%.1f Wheel=%.3fm Track=%.4fm",
            cpr_,
            wheel_diameter_,
            track_width_
        );

        RCLCPP_INFO(
            this->get_logger(),
            "IMU timeout=%.2fs",
            imu_timeout_sec_
        );

        RCLCPP_INFO(
            this->get_logger(),
            "Encoder gap limit=%.2fs",
            max_tick_dt_sec_
        );

        RCLCPP_INFO(
            this->get_logger(),
            "Wheel speed guard=%.2fm/s",
            max_wheel_speed_mps_
        );

        RCLCPP_INFO(
            this->get_logger(),
            "========================================"
        );
    }


private:

    // =========================================================
    // Angle normalize
    // =========================================================

    double normalize_angle(
        double angle
    )
    {
        return std::atan2(
            std::sin(angle),
            std::cos(angle)
        );
    }


    // =========================================================
    // IMU
    // =========================================================

    void imu_callback(
        const sensor_msgs::msg::Imu::SharedPtr msg
    )
    {
        // -----------------------------------------------------
        // NaN / Inf 방어
        // -----------------------------------------------------

        if (
            !std::isfinite(msg->orientation.x) ||
            !std::isfinite(msg->orientation.y) ||
            !std::isfinite(msg->orientation.z) ||
            !std::isfinite(msg->orientation.w) ||
            !std::isfinite(msg->angular_velocity.z)
        )
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "잘못된 IMU 데이터(NaN/Inf) 무시"
            );

            return;
        }


        // -----------------------------------------------------
        // Quaternion 유효성 확인
        // -----------------------------------------------------

        const double norm =
            std::sqrt(
                msg->orientation.x * msg->orientation.x +
                msg->orientation.y * msg->orientation.y +
                msg->orientation.z * msg->orientation.z +
                msg->orientation.w * msg->orientation.w
            );


        if (norm < 1e-6)
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "IMU quaternion norm 비정상"
            );

            return;
        }


        // 정규화
        const double qx =
            msg->orientation.x / norm;

        const double qy =
            msg->orientation.y / norm;

        const double qz =
            msg->orientation.z / norm;

        const double qw =
            msg->orientation.w / norm;


        // -----------------------------------------------------
        // IMU Yaw
        // -----------------------------------------------------

        const double siny_cosp =
            2.0 *
            (
                qw * qz +
                qx * qy
            );


        const double cosy_cosp =
            1.0 -
            2.0 *
            (
                qy * qy +
                qz * qz
            );


        imu_yaw_ =
            std::atan2(
                siny_cosp,
                cosy_cosp
            );


        imu_angular_z_ =
            msg->angular_velocity.z;


        last_imu_time_ =
            this->now();


        imu_received_ =
            true;
    }


    // =========================================================
    // Wheel ticks
    // =========================================================

    void tick_callback(
        const std_msgs::msg::Int32MultiArray::SharedPtr msg
    )
    {
        // -----------------------------------------------------
        // Packet length
        // -----------------------------------------------------

        if (msg->data.size() < 2)
        {
            RCLCPP_WARN(
                this->get_logger(),
                "wheel_ticks 데이터 길이가 부족합니다."
            );

            return;
        }


        const rclcpp::Time current_time =
            this->now();


        const int current_left_tick =
            msg->data[0];


        const int current_right_tick =
            msg->data[1];


        // =====================================================
        // First tick
        // =====================================================

        if (!first_tick_received_)
        {
            prev_left_tick_ =
                current_left_tick;

            prev_right_tick_ =
                current_right_tick;

            last_time_ =
                current_time;

            first_tick_received_ =
                true;


            RCLCPP_INFO(
                this->get_logger(),
                "첫 Encoder 기준값 설정: L=%d R=%d",
                current_left_tick,
                current_right_tick
            );


            publish_odom_and_tf(
                current_time,
                0.0,
                0.0
            );


            return;
        }


        // =====================================================
        // dt
        // =====================================================

        const double dt =
            (
                current_time -
                last_time_
            ).seconds();


        if (dt <= 0.0)
        {
            return;
        }


        // =====================================================
        // Long encoder/serial gap
        //
        // 긴 gap 동안의 tick을 한번에 적분하면
        // 순간적인 velocity / TF jump가 발생할 수 있으므로
        // 현재 값을 새로운 baseline으로 사용.
        // =====================================================

        if (dt > max_tick_dt_sec_)
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Encoder gap %.3fs > %.3fs - tick baseline 재동기화",
                dt,
                max_tick_dt_sec_
            );


            prev_left_tick_ =
                current_left_tick;

            prev_right_tick_ =
                current_right_tick;

            last_time_ =
                current_time;


            publish_odom_and_tf(
                current_time,
                0.0,
                0.0
            );


            return;
        }


        // =====================================================
        // Tick delta
        //
        // int64로 계산하여 signed subtraction overflow 방지.
        // =====================================================

        const int64_t delta_left_tick =
            static_cast<int64_t>(
                current_left_tick
            ) -
            static_cast<int64_t>(
                prev_left_tick_
            );


        const int64_t delta_right_tick =
            static_cast<int64_t>(
                current_right_tick
            ) -
            static_cast<int64_t>(
                prev_right_tick_
            );


        const double delta_left =
            static_cast<double>(
                delta_left_tick
            ) *
            distance_per_tick_;


        const double delta_right =
            static_cast<double>(
                delta_right_tick
            ) *
            distance_per_tick_;


        // =====================================================
        // Tick jump guard
        //
        // STM reset / encoder total reset / corrupted packet 방어.
        // =====================================================

        const double max_delta_distance =
            max_wheel_speed_mps_ *
            dt +
            tick_jump_margin_m_;


        if (
            std::abs(delta_left) >
                max_delta_distance ||
            std::abs(delta_right) >
                max_delta_distance
        )
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "비정상 Encoder jump 무시: "
                "dt=%.3f L=%.3fm R=%.3fm limit=%.3fm",
                dt,
                delta_left,
                delta_right,
                max_delta_distance
            );


            prev_left_tick_ =
                current_left_tick;

            prev_right_tick_ =
                current_right_tick;

            last_time_ =
                current_time;


            publish_odom_and_tf(
                current_time,
                0.0,
                0.0
            );


            return;
        }


        // =====================================================
        // Differential drive odometry
        // =====================================================

        const double d_center =
            (
                delta_right +
                delta_left
            ) /
            2.0;


        const double d_theta_wheel =
            (
                delta_right -
                delta_left
            ) /
            track_width_;


        const double v_x =
            d_center /
            dt;


        const double v_theta_enc =
            d_theta_wheel /
            dt;


        // =====================================================
        // XY integration
        //
        // 회전 중에도 오차를 줄이기 위해
        // 이번 step의 중간 heading 사용.
        // =====================================================

        const double theta_mid =
            theta_ +
            (
                d_theta_wheel *
                0.5
            );


        x_ +=
            d_center *
            std::cos(
                theta_mid
            );


        y_ +=
            d_center *
            std::sin(
                theta_mid
            );


        // =====================================================
        // IMU freshness
        // =====================================================

        bool imu_fresh =
            false;


        if (imu_received_)
        {
            const double imu_age =
                (
                    current_time -
                    last_imu_time_
                ).seconds();


            imu_fresh =
                (
                    imu_age >= 0.0 &&
                    imu_age <=
                        imu_timeout_sec_
                );
        }


        // 상태 변화 로그
        if (
            imu_fresh &&
            !imu_fresh_last_cycle_
        )
        {
            RCLCPP_INFO(
                this->get_logger(),
                "IMU 정상 - Wheel + IMU fusion 사용"
            );
        }
        else if (
            !imu_fresh &&
            imu_fresh_last_cycle_
        )
        {
            RCLCPP_WARN(
                this->get_logger(),
                "IMU timeout - Encoder-only odometry로 전환"
            );
        }


        imu_fresh_last_cycle_ =
            imu_fresh;


        double v_theta =
            v_theta_enc;


        // =====================================================
        // Heading fusion
        // =====================================================

        theta_ +=
            d_theta_wheel;


        if (imu_fresh)
        {
            const double yaw_error =
                normalize_angle(
                    imu_yaw_ -
                    theta_
                );


            theta_ +=
                imu_trust_factor_ *
                yaw_error;


            v_theta =
                (
                    imu_angular_z_ *
                    (
                        1.0 -
                        enc_v_theta_weight_
                    )
                ) +
                (
                    v_theta_enc *
                    enc_v_theta_weight_
                );
        }


        theta_ =
            normalize_angle(
                theta_
            );


        // =====================================================
        // Update baseline
        // =====================================================

        prev_left_tick_ =
            current_left_tick;


        prev_right_tick_ =
            current_right_tick;


        last_time_ =
            current_time;


        // =====================================================
        // Publish
        // =====================================================

        publish_odom_and_tf(
            current_time,
            v_x,
            v_theta
        );
    }


    // =========================================================
    // Publish odometry + TF
    // =========================================================

    void publish_odom_and_tf(
        const rclcpp::Time & current_time,
        double v_x,
        double v_theta
    )
    {
        tf2::Quaternion q;


        q.setRPY(
            0.0,
            0.0,
            theta_
        );


        // =====================================================
        // TF
        // =====================================================

        geometry_msgs::msg::TransformStamped t;


        t.header.stamp =
            current_time;


        t.header.frame_id =
            "robot1_odom";


        t.child_frame_id =
            "robot1_base";


        t.transform.translation.x =
            x_;

        t.transform.translation.y =
            y_;

        t.transform.translation.z =
            0.0;


        t.transform.rotation.x =
            q.x();

        t.transform.rotation.y =
            q.y();

        t.transform.rotation.z =
            q.z();

        t.transform.rotation.w =
            q.w();


        tf_broadcaster_->sendTransform(
            t
        );


        // =====================================================
        // Odometry
        // =====================================================

        nav_msgs::msg::Odometry odom;


        odom.header.stamp =
            current_time;


        odom.header.frame_id =
            "robot1_odom";


        odom.child_frame_id =
            "robot1_base";


        odom.pose.pose.position.x =
            x_;

        odom.pose.pose.position.y =
            y_;

        odom.pose.pose.position.z =
            0.0;


        odom.pose.pose.orientation =
            t.transform.rotation;


        odom.twist.twist.linear.x =
            v_x;


        odom.twist.twist.angular.z =
            v_theta;


        odom_pub_->publish(
            odom
        );
    }


    // =========================================================
    // Parameters
    // =========================================================

    double cpr_;
    double wheel_diameter_;
    double track_width_;
    double distance_per_tick_;

    double imu_trust_factor_;
    double enc_v_theta_weight_;

    double imu_timeout_sec_;

    double max_tick_dt_sec_;
    double max_wheel_speed_mps_;
    double tick_jump_margin_m_;


    // =========================================================
    // Pose
    // =========================================================

    double x_;
    double y_;
    double theta_;


    // =========================================================
    // IMU
    // =========================================================

    double imu_yaw_;
    double imu_angular_z_;

    bool imu_received_;
    bool imu_fresh_last_cycle_;

    rclcpp::Time
        last_imu_time_;


    // =========================================================
    // Encoder
    // =========================================================

    bool first_tick_received_;

    int prev_left_tick_;
    int prev_right_tick_;

    rclcpp::Time
        last_time_;


    // =========================================================
    // ROS Interfaces
    // =========================================================

    rclcpp::Publisher<
        nav_msgs::msg::Odometry
    >::SharedPtr
        odom_pub_;


    std::unique_ptr<
        tf2_ros::TransformBroadcaster
    >
        tf_broadcaster_;


    rclcpp::Subscription<
        std_msgs::msg::Int32MultiArray
    >::SharedPtr
        tick_sub_;


    rclcpp::Subscription<
        sensor_msgs::msg::Imu
    >::SharedPtr
        imu_sub_;
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
            OdomPublisher
        >();


    rclcpp::spin(
        node
    );


    rclcpp::shutdown();


    return 0;
}
