#include <cmath>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>

#include <nlohmann/json.hpp>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"


using json = nlohmann::json;


class DestinationResolver : public rclcpp::Node
{
public:
    DestinationResolver()
    : Node("destination_resolver")
    {
        // ======================================================
        // destinations.json 기본 경로
        //
        // install/robot1_nav/share/robot1_nav/config/
        // destinations.json
        // ======================================================

        const std::string default_json_path =
            ament_index_cpp::get_package_share_directory(
                "robot1_nav"
            )
            + "/config/destinations.json";


        destinations_file_ =
            this->declare_parameter<std::string>(
                "destinations_file",
                default_json_path
            );


        // ======================================================
        // JSON Load
        // ======================================================

        load_destinations();


        // ======================================================
        // Publisher
        // ======================================================

        pose_pub_ =
            this->create_publisher<
                geometry_msgs::msg::PoseStamped
            >(
                "/destination_pose",
                10
            );


        // ======================================================
        // Voice Subscriber
        //
        // /voice_command
        // data: "내과"
        // ======================================================

        voice_sub_ =
            this->create_subscription<
                std_msgs::msg::String
            >(
                "/voice_command",
                10,

                std::bind(
                    &DestinationResolver::voice_callback,
                    this,
                    std::placeholders::_1
                )
            );


        RCLCPP_INFO(
            this->get_logger(),
            "Destination Resolver started"
        );

        RCLCPP_INFO(
            this->get_logger(),
            "JSON: %s",
            destinations_file_.c_str()
        );

        RCLCPP_INFO(
            this->get_logger(),
            "Subscribe : /voice_command"
        );

        RCLCPP_INFO(
            this->get_logger(),
            "Publish   : /destination_pose"
        );
    }


private:

    // ==========================================================
    // destinations.json Load
    // ==========================================================

    void load_destinations()
    {
        std::ifstream file(destinations_file_);


        if (!file.is_open())
        {
            throw std::runtime_error(
                "Cannot open destinations file: "
                + destinations_file_
            );
        }


        try
        {
            destination_db_ = json::parse(file);
        }
        catch (const json::exception & e)
        {
            throw std::runtime_error(
                std::string("JSON parse error: ")
                + e.what()
            );
        }


        // ------------------------------------------------------
        // destinations 확인
        // ------------------------------------------------------

        if (!destination_db_.contains("destinations"))
        {
            throw std::runtime_error(
                "destinations.json has no "
                "'destinations' object"
            );
        }


        if (!destination_db_["destinations"].is_object())
        {
            throw std::runtime_error(
                "'destinations' must be a JSON object"
            );
        }


        // ------------------------------------------------------
        // frame_id
        // ------------------------------------------------------

        frame_id_ =
            destination_db_.value(
                "frame_id",
                std::string("map")
            );


        if (frame_id_.empty())
        {
            frame_id_ = "map";
        }


        RCLCPP_INFO(
            this->get_logger(),
            "Loaded %zu destinations, frame_id=%s",
            destination_db_["destinations"].size(),
            frame_id_.c_str()
        );
    }


    // ==========================================================
    // /voice_command Callback
    // ==========================================================

    void voice_callback(
        const std_msgs::msg::String::SharedPtr msg
    )
    {
        const std::string destination_name =
            msg->data;


        // ------------------------------------------------------
        // Empty command
        // ------------------------------------------------------

        if (destination_name.empty())
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Received empty destination"
            );

            return;
        }


        RCLCPP_INFO(
            this->get_logger(),
            "Destination command: [%s]",
            destination_name.c_str()
        );


        const auto & destinations =
            destination_db_["destinations"];


        // ------------------------------------------------------
        // Unknown destination
        // ------------------------------------------------------

        if (!destinations.contains(destination_name))
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Unknown destination: [%s]",
                destination_name.c_str()
            );

            return;
        }


        const auto & destination =
            destinations.at(destination_name);


        // ------------------------------------------------------
        // x / y 확인
        // ------------------------------------------------------

        if (
            !destination.contains("x") ||
            !destination.contains("y")
        )
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "Destination [%s] has no x/y",
                destination_name.c_str()
            );

            return;
        }


        double x = 0.0;
        double y = 0.0;

        double q_z = 0.0;
        double q_w = 1.0;

        double yaw = 0.0;


        // ======================================================
        // JSON 데이터 읽기
        //
        // 우선순위:
        //
        // 1. q_z + q_w
        // 2. yaw
        // 3. 둘 다 없으면 yaw=0
        //
        // RViz 2D Goal Pose에서 얻은 quaternion을
        // 그대로 저장하는 방식을 기본으로 사용한다.
        // ======================================================

        try
        {
            x = destination.at("x").get<double>();
            y = destination.at("y").get<double>();


            // --------------------------------------------------
            // Quaternion 직접 저장 방식
            // --------------------------------------------------

            const bool has_q_z =
                destination.contains("q_z");

            const bool has_q_w =
                destination.contains("q_w");


            // 하나만 있으면 잘못된 데이터
            if (has_q_z != has_q_w)
            {
                RCLCPP_ERROR(
                    this->get_logger(),
                    "Destination [%s]: "
                    "q_z and q_w must both exist",
                    destination_name.c_str()
                );

                return;
            }


            if (has_q_z && has_q_w)
            {
                q_z =
                    destination.at("q_z").get<double>();

                q_w =
                    destination.at("q_w").get<double>();


                // ----------------------------------------------
                // Quaternion normalization
                //
                // 2D이므로 x=y=0,
                // z/w만 사용한다.
                // ----------------------------------------------

                const double norm =
                    std::hypot(q_z, q_w);


                if (norm < 1e-9)
                {
                    RCLCPP_ERROR(
                        this->get_logger(),
                        "Destination [%s] has invalid "
                        "quaternion q_z=%.6f q_w=%.6f",
                        destination_name.c_str(),
                        q_z,
                        q_w
                    );

                    return;
                }


                q_z /= norm;
                q_w /= norm;


                // 로그용 yaw 계산
                yaw =
                    2.0 * std::atan2(
                        q_z,
                        q_w
                    );
            }

            // --------------------------------------------------
            // 기존 yaw 방식도 호환
            // --------------------------------------------------

            else if (destination.contains("yaw"))
            {
                yaw =
                    destination.at("yaw").get<double>();


                q_z =
                    std::sin(
                        yaw * 0.5
                    );

                q_w =
                    std::cos(
                        yaw * 0.5
                    );
            }

            // --------------------------------------------------
            // Orientation 없는 기존 목적지
            //
            // backward compatibility를 위해 yaw=0으로
            // 발행은 허용한다.
            //
            // 단, SmacPlannerLattice에서는 goal yaw가
            // 중요하므로 경고를 출력한다.
            // --------------------------------------------------

            else
            {
                yaw = 0.0;

                q_z = 0.0;
                q_w = 1.0;


                RCLCPP_WARN(
                    this->get_logger(),
                    "Destination [%s] has no orientation. "
                    "Using yaw=0.0. "
                    "SmacPlannerLattice may fail if this "
                    "goal orientation is not reachable.",
                    destination_name.c_str()
                );
            }
        }
        catch (const json::exception & e)
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "Invalid destination data [%s]: %s",
                destination_name.c_str(),
                e.what()
            );

            return;
        }


        // ======================================================
        // PoseStamped
        // ======================================================

        geometry_msgs::msg::PoseStamped pose;


        pose.header.stamp =
            this->get_clock()->now();

        pose.header.frame_id =
            frame_id_;


        // ------------------------------------------------------
        // Position
        // ------------------------------------------------------

        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = 0.0;


        // ------------------------------------------------------
        // Orientation
        //
        // 2D navigation:
        // roll  = 0
        // pitch = 0
        // ------------------------------------------------------

        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;

        pose.pose.orientation.z = q_z;
        pose.pose.orientation.w = q_w;


        // ======================================================
        // Publish
        // ======================================================

        pose_pub_->publish(pose);


        RCLCPP_INFO(
            this->get_logger(),
            "Resolved [%s] -> "
            "x=%.3f, y=%.3f, "
            "yaw=%.3f rad, "
            "q_z=%.6f, q_w=%.6f",
            destination_name.c_str(),
            x,
            y,
            yaw,
            q_z,
            q_w
        );
    }


    // ==========================================================
    // ROS Interfaces
    // ==========================================================

    rclcpp::Subscription<
        std_msgs::msg::String
    >::SharedPtr voice_sub_;


    rclcpp::Publisher<
        geometry_msgs::msg::PoseStamped
    >::SharedPtr pose_pub_;


    // ==========================================================
    // Destination DB
    // ==========================================================

    std::string destinations_file_;

    std::string frame_id_;

    json destination_db_;
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


    try
    {
        rclcpp::spin(
            std::make_shared<
                DestinationResolver
            >()
        );
    }
    catch (const std::exception & e)
    {
        RCLCPP_FATAL(
            rclcpp::get_logger(
                "destination_resolver"
            ),
            "%s",
            e.what()
        );


        rclcpp::shutdown();

        return 1;
    }


    rclcpp::shutdown();

    return 0;
}
