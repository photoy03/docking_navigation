#include <chrono>
#include <memory>
#include <cmath>

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
    : Node("robot1_odom_node"), x_(0.0), y_(0.0), theta_(0.0), imu_yaw_(0.0), use_imu_(false),
      prev_left_tick_(0), prev_right_tick_(0)
    {
        // 1. ROS 2 파라미터 선언 및 최후의 안전 기본값(Default) 지정
        this->declare_parameter<double>("cpr", 4000.0);
        this->declare_parameter<double>("wheel_diameter", 0.15);
        this->declare_parameter<double>("track_width", 0.45);

        // 2. YAML 또는 외부에서 입력된 실시간 파라미터 획득
        cpr_ = this->get_parameter("cpr").as_double();
        wheel_diameter_ = this->get_parameter("wheel_diameter").as_double();
        track_width_ = this->get_parameter("track_width").as_double();
        
        // 3. 획득한 물리 파라미터를 기반으로 기구학 상수 계산
        distance_per_tick_ = (M_PI * wheel_diameter_) / cpr_;

        RCLCPP_INFO(this->get_logger(), "========================================");
        RCLCPP_INFO(this->get_logger(), " 하드웨어 파라미터 동적 바인딩 성공!");
        RCLCPP_INFO(this->get_logger(), " 모터 CPR: %.1f", cpr_);
        RCLCPP_INFO(this->get_logger(), " 바퀴 지름: %.3f m", wheel_diameter_);
        RCLCPP_INFO(this->get_logger(), " 윤거 (Track Width): %.3f m", track_width_);
        RCLCPP_INFO(this->get_logger(), " 틱당 이동거리: %.8f m", distance_per_tick_);
        RCLCPP_INFO(this->get_logger(), "========================================");

        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/robot1/odom", 10);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        tick_sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "/robot1/wheel_ticks", 10, std::bind(&OdomPublisher::tick_callback, this, std::placeholders::_1));
        
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/robot1/imu/data", 10, std::bind(&OdomPublisher::imu_callback, this, std::placeholders::_1));

        last_time_ = this->now();
    }

private:
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        double siny_cosp = 2.0 * (msg->orientation.w * msg->orientation.z + msg->orientation.x * msg->orientation.y);
        double cosy_cosp = 1.0 - 2.0 * (msg->orientation.y * msg->orientation.y + msg->orientation.z * msg->orientation.z);
        imu_yaw_ = std::atan2(siny_cosp, cosy_cosp);
        use_imu_ = true;
    }

    void tick_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
    {
        rclcpp::Time current_time = this->now();
        double dt = (current_time - last_time_).seconds();

        if (dt <= 0.0) return;

        int current_left_tick = msg->data[0];
        int current_right_tick = msg->data[1];

        double delta_left = (current_left_tick - prev_left_tick_) * distance_per_tick_;
        double delta_right = (current_right_tick - prev_right_tick_) * distance_per_tick_;

        double d_center = (delta_right + delta_left) / 2.0;
        double d_theta = (delta_right - delta_left) / track_width_;

        x_ += d_center * std::cos(theta_);
        y_ += d_center * std::sin(theta_);

        if (use_imu_) {
            theta_ = imu_yaw_;
        } else {
            theta_ += d_theta;
        }

        double v_x = d_center / dt;
        double v_theta = d_theta / dt;

        prev_left_tick_ = current_left_tick;
        prev_right_tick_ = current_right_tick;
        last_time_ = current_time;

        publish_odom_and_tf(current_time, v_x, v_theta);
    }

    void publish_odom_and_tf(const rclcpp::Time & current_time, double v_x, double v_theta)
    {
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, theta_);

        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = current_time;
        t.header.frame_id = "robot1_odom";
        t.child_frame_id = "robot1_base";
        t.transform.translation.x = x_;
        t.transform.translation.y = y_;
        t.transform.translation.z = 0.0;
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();
        tf_broadcaster_->sendTransform(t);

        nav_msgs::msg::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "robot1_odom";
        odom.child_frame_id = "robot1_base";
        odom.pose.pose.position.x = x_;
        odom.pose.pose.position.y = y_;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = t.transform.rotation;
        odom.twist.twist.linear.x = v_x;
        odom.twist.twist.angular.z = v_theta;
        odom_pub_->publish(odom);
    }

    double cpr_, wheel_diameter_, track_width_, distance_per_tick_;
    double x_, y_, theta_, imu_yaw_;
    bool use_imu_;
    int prev_left_tick_, prev_right_tick_;
    rclcpp::Time last_time_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr tick_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OdomPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
