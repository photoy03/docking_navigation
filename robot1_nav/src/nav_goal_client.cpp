#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"

#include "std_msgs/msg/bool.hpp"

#include "nav2_msgs/action/navigate_to_pose.hpp"


using namespace std::chrono_literals;


class NavGoalClient : public rclcpp::Node
{
public:
    using NavigateToPose =
        nav2_msgs::action::NavigateToPose;

    using GoalHandleNavigate =
        rclcpp_action::ClientGoalHandle<
            NavigateToPose
        >;


    NavGoalClient()
    : Node("nav_goal_client"),
      goal_request_in_flight_(false),
      cancel_pending_(false)
    {
        // =====================================================
        // Action Client
        // =====================================================

        action_client_ =
            rclcpp_action::create_client<
                NavigateToPose
            >(
                this,
                "/navigate_to_pose"
            );


        // =====================================================
        // Approved Goal
        //
        // mode_manager
        //      ↓
        // /nav_goal_pose
        //      ↓
        // Nav2 NavigateToPose
        // =====================================================

        goal_sub_ =
            this->create_subscription<
                geometry_msgs::msg::PoseStamped
            >(
                "/nav_goal_pose",
                10,

                std::bind(
                    &NavGoalClient::goal_callback,
                    this,
                    std::placeholders::_1
                )
            );


        // =====================================================
        // Emergency Goal Cancel
        // =====================================================

        cancel_sub_ =
            this->create_subscription<
                std_msgs::msg::Bool
            >(
                "/nav/cancel",
                10,

                std::bind(
                    &NavGoalClient::cancel_callback,
                    this,
                    std::placeholders::_1
                )
            );


        // =====================================================
        // Finished
        //
        // true 1회:
        // SUCCEEDED / ABORTED / CANCELED / REJECTED /
        // action server unavailable
        // =====================================================

        finished_pub_ =
            this->create_publisher<
                std_msgs::msg::Bool
            >(
                "/nav/finished",
                10
            );


        RCLCPP_INFO(
            this->get_logger(),
            "========================================"
        );

        RCLCPP_INFO(
            this->get_logger(),
            "Nav Goal Client START"
        );

        RCLCPP_INFO(
            this->get_logger(),
            "Subscribe : /nav_goal_pose"
        );

        RCLCPP_INFO(
            this->get_logger(),
            "Cancel    : /nav/cancel"
        );

        RCLCPP_INFO(
            this->get_logger(),
            "Action    : /navigate_to_pose"
        );

        RCLCPP_INFO(
            this->get_logger(),
            "========================================"
        );
    }


private:

    // =========================================================
    // Goal
    // =========================================================

    void goal_callback(
        const geometry_msgs::msg::PoseStamped::SharedPtr msg
    )
    {
        RCLCPP_INFO(
            this->get_logger(),
            "Nav Goal received: "
            "frame=%s, x=%.3f, y=%.3f",
            msg->header.frame_id.c_str(),
            msg->pose.position.x,
            msg->pose.position.y
        );


        // -----------------------------------------------------
        // 이미 active goal 또는 send request 진행 중
        // -----------------------------------------------------

        if (
            current_goal_handle_ ||
            goal_request_in_flight_
        )
        {
            RCLCPP_WARN(
                this->get_logger(),
                "현재 Navigation Goal 진행 중 - 새 Goal 무시"
            );

            return;
        }


        // -----------------------------------------------------
        // Action Server
        // -----------------------------------------------------

        if (
            !action_client_->
                wait_for_action_server(
                    2s
                )
        )
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "/navigate_to_pose action server is not available"
            );


            publish_finished();

            return;
        }


        // -----------------------------------------------------
        // Goal
        // -----------------------------------------------------

        NavigateToPose::Goal
            goal;


        goal.pose =
            *msg;


        // 혹시 stamp가 0인 입력이 들어오면 현재 시간 사용
        if (
            goal.pose.header.stamp.sec == 0 &&
            goal.pose.header.stamp.nanosec == 0
        )
        {
            goal.pose.header.stamp =
                this->now();
        }


        RCLCPP_INFO(
            this->get_logger(),
            "Sending NavigateToPose Goal..."
        );


        goal_request_in_flight_ =
            true;

        cancel_pending_ =
            false;


        // =====================================================
        // Send Goal Options
        //
        // ROS2 Humble:
        // goal_response_callback argument는
        // GoalHandle::SharedPtr
        // =====================================================

        auto send_goal_options =
            rclcpp_action::Client<
                NavigateToPose
            >::SendGoalOptions();


        // -----------------------------------------------------
        // Goal Response
        // -----------------------------------------------------

        send_goal_options.goal_response_callback =
            [this](
                GoalHandleNavigate::SharedPtr
                    goal_handle
            )
            {
                goal_request_in_flight_ =
                    false;


                if (!goal_handle)
                {
                    RCLCPP_ERROR(
                        this->get_logger(),
                        "Nav2 Goal REJECTED"
                    );


                    current_goal_handle_.reset();

                    cancel_pending_ =
                        false;


                    publish_finished();

                    return;
                }


                current_goal_handle_ =
                    goal_handle;


                RCLCPP_INFO(
                    this->get_logger(),
                    "Nav2 Goal ACCEPTED"
                );


                // ---------------------------------------------
                // Emergency cancel이 Goal accepted보다
                // 먼저 들어온 race 대응.
                // ---------------------------------------------

                if (cancel_pending_)
                {
                    RCLCPP_WARN(
                        this->get_logger(),
                        "Goal accepted 직후 pending cancel 실행"
                    );


                    cancel_current_goal();
                }
            };


        // -----------------------------------------------------
        // Feedback
        // -----------------------------------------------------

        send_goal_options.feedback_callback =
            [this](
                GoalHandleNavigate::SharedPtr,
                const std::shared_ptr<
                    const NavigateToPose::Feedback
                > feedback
            )
            {
                if (!feedback)
                {
                    return;
                }


                RCLCPP_INFO_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    1000,
                    "Navigating... "
                    "distance_remaining=%.2f m, "
                    "recoveries=%d",
                    feedback->distance_remaining,
                    static_cast<int>(
                        feedback->number_of_recoveries
                    )
                );
            };


        // -----------------------------------------------------
        // Result
        // -----------------------------------------------------

        send_goal_options.result_callback =
            [this](
                const GoalHandleNavigate::WrappedResult &
                    result
            )
            {
                switch (result.code)
                {
                    case rclcpp_action::ResultCode::SUCCEEDED:

                        RCLCPP_INFO(
                            this->get_logger(),
                            "Navigation SUCCEEDED"
                        );

                        break;


                    case rclcpp_action::ResultCode::ABORTED:

                        RCLCPP_ERROR(
                            this->get_logger(),
                            "Navigation ABORTED"
                        );

                        break;


                    case rclcpp_action::ResultCode::CANCELED:

                        RCLCPP_WARN(
                            this->get_logger(),
                            "Navigation CANCELED"
                        );

                        break;


                    default:

                        RCLCPP_ERROR(
                            this->get_logger(),
                            "Navigation result UNKNOWN"
                        );

                        break;
                }


                current_goal_handle_.reset();

                goal_request_in_flight_ =
                    false;

                cancel_pending_ =
                    false;


                publish_finished();
            };


        // =====================================================
        // Send
        // =====================================================

        action_client_->
            async_send_goal(
                goal,
                send_goal_options
            );
    }


    // =========================================================
    // Cancel
    // =========================================================

    void cancel_callback(
        const std_msgs::msg::Bool::SharedPtr msg
    )
    {
        if (!msg->data)
        {
            return;
        }


        RCLCPP_WARN(
            this->get_logger(),
            "/nav/cancel 요청 수신"
        );


        // -----------------------------------------------------
        // Goal request가 아직 Nav2로부터 accepted/rejected
        // 응답을 기다리는 중인 경우
        // -----------------------------------------------------

        if (
            goal_request_in_flight_ &&
            !current_goal_handle_
        )
        {
            cancel_pending_ =
                true;


            RCLCPP_WARN(
                this->get_logger(),
                "Goal response 대기 중 - cancel pending"
            );


            return;
        }


        // -----------------------------------------------------
        // Active Goal 없음
        // -----------------------------------------------------

        if (!current_goal_handle_)
        {
            RCLCPP_INFO(
                this->get_logger(),
                "Cancel 대상 NavigateToPose Goal 없음"
            );


            return;
        }


        cancel_current_goal();
    }


    // =========================================================
    // Cancel current Nav2 Goal
    // =========================================================

    void cancel_current_goal()
    {
        if (!current_goal_handle_)
        {
            return;
        }


        cancel_pending_ =
            false;


        RCLCPP_WARN(
            this->get_logger(),
            "Current NavigateToPose Goal CANCEL 요청"
        );


        action_client_->
            async_cancel_goal(
                current_goal_handle_
            );
    }


    // =========================================================
    // Finished Publisher
    // =========================================================

    void publish_finished()
    {
        std_msgs::msg::Bool
            msg;


        msg.data =
            true;


        finished_pub_->publish(
            msg
        );
    }


    // =========================================================
    // ROS
    // =========================================================

    rclcpp_action::Client<
        NavigateToPose
    >::SharedPtr action_client_;


    rclcpp::Subscription<
        geometry_msgs::msg::PoseStamped
    >::SharedPtr goal_sub_;


    rclcpp::Subscription<
        std_msgs::msg::Bool
    >::SharedPtr cancel_sub_;


    rclcpp::Publisher<
        std_msgs::msg::Bool
    >::SharedPtr finished_pub_;


    // =========================================================
    // Goal State
    // =========================================================

    GoalHandleNavigate::SharedPtr
        current_goal_handle_;


    bool goal_request_in_flight_;

    bool cancel_pending_;
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
            NavGoalClient
        >();


    rclcpp::spin(
        node
    );


    rclcpp::shutdown();


    return 0;
}
