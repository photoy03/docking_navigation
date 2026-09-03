#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"

#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"


using namespace std::chrono_literals;


class ModeManager : public rclcpp::Node
{
public:
    ModeManager()
    : Node("mode_manager"),
      mcu_state_(-1),
      docked_(false),
      emergency_(false),
      has_pending_goal_(false),
      phase_(Phase::IDLE),
      admittance_retry_count_(0),
      admittance_retry_exhausted_(false)
    {
        // =====================================================
        // State QoS
        // =====================================================

        rclcpp::QoS state_qos(1);

        state_qos
            .reliable()
            .transient_local();


        // =====================================================
        // Destination
        //
        // destination_resolver
        //      ↓
        // /destination_pose
        //      ↓
        // mode_manager
        // =====================================================

        destination_sub_ =
            this->create_subscription<
                geometry_msgs::msg::PoseStamped
            >(
                "/destination_pose",
                10,

                std::bind(
                    &ModeManager::destination_callback,
                    this,
                    std::placeholders::_1
                )
            );


        // =====================================================
        // MCU State
        // =====================================================

        state_sub_ =
            this->create_subscription<
                std_msgs::msg::Int32
            >(
                "/mcu/state",
                state_qos,

                std::bind(
                    &ModeManager::state_callback,
                    this,
                    std::placeholders::_1
                )
            );


        // =====================================================
        // Docked
        // =====================================================

        docked_sub_ =
            this->create_subscription<
                std_msgs::msg::Bool
            >(
                "/mcu/docked",
                state_qos,

                std::bind(
                    &ModeManager::docked_callback,
                    this,
                    std::placeholders::_1
                )
            );


        // =====================================================
        // Emergency
        // =====================================================

        emergency_sub_ =
            this->create_subscription<
                std_msgs::msg::Bool
            >(
                "/mcu/emergency",
                state_qos,

                std::bind(
                    &ModeManager::emergency_callback,
                    this,
                    std::placeholders::_1
                )
            );


        // =====================================================
        // Nav finished
        //
        // success / aborted / canceled / rejected
        // =====================================================

        nav_finished_sub_ =
            this->create_subscription<
                std_msgs::msg::Bool
            >(
                "/nav/finished",
                10,

                std::bind(
                    &ModeManager::nav_finished_callback,
                    this,
                    std::placeholders::_1
                )
            );


        // =====================================================
        // MCU Mode command
        // =====================================================

        mode_cmd_pub_ =
            this->create_publisher<
                std_msgs::msg::Int32
            >(
                "/mcu/mode_cmd",
                10
            );


        // =====================================================
        // Approved Nav Goal
        // =====================================================

        nav_goal_pub_ =
            this->create_publisher<
                geometry_msgs::msg::PoseStamped
            >(
                "/nav_goal_pose",
                10
            );


        // =====================================================
        // Nav Goal Cancel
        //
        // Emergency 발생 시 현재 NavigateToPose만 취소.
        // Nav2 lifecycle은 내리지 않는다.
        // =====================================================

        nav_cancel_pub_ =
            this->create_publisher<
                std_msgs::msg::Bool
            >(
                "/nav/cancel",
                10
            );


        // =====================================================
        // Timer
        // =====================================================

        timer_ =
            this->create_wall_timer(
                100ms,

                std::bind(
                    &ModeManager::timer_callback,
                    this
                )
            );


        RCLCPP_INFO(
            this->get_logger(),
            "========================================"
        );

        RCLCPP_INFO(
            this->get_logger(),
            "Mode Manager START"
        );

        RCLCPP_INFO(
            this->get_logger(),
            "Destination -> AUTO -> Nav2 -> ADMITTANCE"
        );

        RCLCPP_INFO(
            this->get_logger(),
            "Emergency -> Nav cancel -> STM recovery -> STATE1"
        );

        RCLCPP_INFO(
            this->get_logger(),
            "========================================"
        );
    }


private:

    // =========================================================
    // Phase
    // =========================================================

    enum class Phase
    {
        IDLE,

        WAIT_AUTO,

        NAVIGATING,

        WAIT_ADMITTANCE,

        EMERGENCY
    };


    // =========================================================
    // Destination
    // =========================================================

    void destination_callback(
        const geometry_msgs::msg::PoseStamped::SharedPtr msg
    )
    {
        RCLCPP_INFO(
            this->get_logger(),
            "Destination 요청: x=%.3f y=%.3f",
            msg->pose.position.x,
            msg->pose.position.y
        );


        // -----------------------------------------------------
        // Docking 전
        // -----------------------------------------------------

        if (!docked_)
        {
            RCLCPP_WARN(
                this->get_logger(),
                "목적지 거부: 아직 도킹되지 않음"
            );

            return;
        }
        
        // -----------------------------------------------------
// MCU STOP 상태에서는 목적지 금지
// -----------------------------------------------------
if (mcu_state_ == 0)
{
    RCLCPP_ERROR(
        this->get_logger(),
        "목적지 거부: MCU STATE0 STOP"
    );
    return;
}


        // -----------------------------------------------------
        // Emergency
        // -----------------------------------------------------

        if (emergency_)
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "목적지 거부: Emergency 상태"
            );

            return;
        }


        if (phase_ == Phase::EMERGENCY)
        {
            RCLCPP_WARN(
                this->get_logger(),
                "목적지 거부: Emergency recovery 완료 전"
            );

            return;
        }


        // -----------------------------------------------------
        // Busy
        // -----------------------------------------------------

        if (phase_ != Phase::IDLE)
        {
            RCLCPP_WARN(
                this->get_logger(),
                "목적지 거부: Navigation 또는 모드 전환 진행 중"
            );

            return;
        }


        // -----------------------------------------------------
        // Pending Goal 저장
        // -----------------------------------------------------

        pending_goal_ =
            *msg;

        has_pending_goal_ =
            true;


        // -----------------------------------------------------
        // 이미 AUTO이면 바로 전달
        // -----------------------------------------------------

        if (mcu_state_ == 2)
        {
            RCLCPP_WARN(
                this->get_logger(),
                "MCU 이미 STATE2 - Goal 바로 전달"
            );


            send_nav_goal();

            return;
        }


        // -----------------------------------------------------
        // STATE1 -> M2 -> S2 대기
        // -----------------------------------------------------

        request_mode(
            2
        );


        phase_ =
            Phase::WAIT_AUTO;


        mode_request_time_ =
            this->now();


        RCLCPP_INFO(
            this->get_logger(),
            "AUTO 전환 요청 - MCU S,2 대기"
        );
    }


    // =========================================================
    // MCU State
    // =========================================================

    void state_callback(
        const std_msgs::msg::Int32::SharedPtr msg
    )
    {
        const int previous_state =
            mcu_state_;


        mcu_state_ =
            msg->data;


        if (
            previous_state !=
            mcu_state_
        )
        {
            RCLCPP_INFO(
                this->get_logger(),
                "MCU STATE 변경: %d -> %d",
                previous_state,
                mcu_state_
            );
        }
        
        
        // =====================================================
// Unexpected MCU STOP
//
// Navigation 또는 AUTO 전환 중 STM이 STATE0가 되면
// 현재 Nav Goal / pending goal을 즉시 폐기한다.
//
// 중요:
// phase를 먼저 NAVIGATING에서 벗어나게 해야
// Nav cancel result로 /nav/finished가 돌아와도
// nav_finished_callback()이 M1을 다시 보내지 않는다.
// =====================================================
if (
    mcu_state_ == 0 &&
    (
        phase_ == Phase::NAVIGATING ||
        phase_ == Phase::WAIT_AUTO
    )
)
{
    const bool nav_was_active =
        (phase_ == Phase::NAVIGATING);

    RCLCPP_ERROR(
        this->get_logger(),
        "MCU STATE0 감지 - AUTO/Navigation 중단"
    );

    // pending destination 폐기
    has_pending_goal_ = false;

    // cancel 결과 callback보다 먼저 phase를 변경해야 함
    phase_ = Phase::IDLE;

    // 실제 Nav2 goal이 전달된 경우에만 cancel
    if (nav_was_active)
    {
        request_nav_cancel();

        RCLCPP_ERROR(
            this->get_logger(),
            "Navigation 중 MCU STATE0 -> Nav Goal 즉시 Cancel"
        );
    }
    else
    {
        RCLCPP_ERROR(
            this->get_logger(),
            "AUTO 전환 중 MCU STATE0 -> Pending Goal 폐기"
        );
    }

    return;
}


        // =====================================================
        // Emergency recovery 완료
        //
        // STM:
        // E,0
        // S,1
        //
        // 둘 다 확인되어야 IDLE 복귀
        // =====================================================

        if (
            phase_ == Phase::EMERGENCY &&
            !emergency_ &&
            mcu_state_ == 1
        )
        {
            phase_ =
                Phase::IDLE;


            has_pending_goal_ =
                false;


            RCLCPP_INFO(
                this->get_logger(),
                "========================================"
            );

            RCLCPP_INFO(
                this->get_logger(),
                "Emergency Recovery 완료"
            );

            RCLCPP_INFO(
                this->get_logger(),
                "MCU STATE1 ADMITTANCE"
            );

            RCLCPP_INFO(
                this->get_logger(),
                "새 목적지 입력 대기"
            );

            RCLCPP_INFO(
                this->get_logger(),
                "========================================"
            );


            return;
        }


        // =====================================================
        // AUTO 확인
        // =====================================================

        if (
            phase_ == Phase::WAIT_AUTO &&
            mcu_state_ == 2 &&
            has_pending_goal_ &&
            !emergency_
        )
        {
            RCLCPP_INFO(
                this->get_logger(),
                "MCU STATE2 AUTO 확인"
            );


            send_nav_goal();

            return;
        }


        // =====================================================
        // ADMITTANCE 복귀
        //
        // 정상 navigation 종료 후
        // M1 -> S1
        // =====================================================

        if (
            phase_ == Phase::WAIT_ADMITTANCE &&
            mcu_state_ == 1
        )
        {
            phase_ =
                Phase::IDLE;
                
                
            admittance_retry_count_ =
    		0;

	    admittance_retry_exhausted_ =
    		false;


            RCLCPP_INFO(
                this->get_logger(),
                "MCU STATE1 ADMITTANCE 복귀 완료"
            );

            RCLCPP_INFO(
                this->get_logger(),
                "다음 목적지 입력 대기"
            );


            return;
        }
    }


    // =========================================================
    // Docked
    // =========================================================

    void docked_callback(
        const std_msgs::msg::Bool::SharedPtr msg
    )
    {
        const bool previous =
            docked_;


        docked_ =
            msg->data;


        if (
            previous !=
            docked_
        )
        {
            RCLCPP_INFO(
                this->get_logger(),
                "Docked = %s",
                docked_
                    ? "true"
                    : "false"
            );
        }
    }


// =========================================================
// Emergency
// =========================================================

void emergency_callback(
    const std_msgs::msg::Bool::SharedPtr msg
)
{
    const bool previous_emergency =
        emergency_;

    emergency_ =
        msg->data;


    // =====================================================
    // Emergency SET
    // =====================================================

    if (
        emergency_ &&
        !previous_emergency
    )
    {
        RCLCPP_ERROR(
            this->get_logger(),
            "========================================"
        );

        RCLCPP_ERROR(
            this->get_logger(),
            "MCU EMERGENCY 감지"
        );

        RCLCPP_ERROR(
            this->get_logger(),
            "========================================"
        );


        const bool nav_was_active =
            (
                phase_ == Phase::NAVIGATING ||
                phase_ == Phase::WAIT_AUTO
            );


        // -------------------------------------------------
        // Pending goal 폐기
        // -------------------------------------------------

        has_pending_goal_ =
            false;


        // -------------------------------------------------
        // 반드시 Nav cancel보다 먼저
        // EMERGENCY phase 확정
        //
        // 이후 cancel 결과로 /nav/finished가 들어와도
        // M1을 보내지 않게 함.
        // -------------------------------------------------

        phase_ =
            Phase::EMERGENCY;


        // -------------------------------------------------
        // Navigation 또는 AUTO 전환 중이었다면
        // 현재 Nav Goal 취소
        // -------------------------------------------------

        if (nav_was_active)
        {
            request_nav_cancel();
        }


        return;
    }


    // =====================================================
    // Emergency CLEAR
    //
    // STM:
    // E,0
    //
    // 아직 S,1이 안 왔을 수도 있으므로
    // STATE1까지 확인해야 Recovery 완료.
    // =====================================================

    if (
        !emergency_ &&
        previous_emergency
    )
    {
        RCLCPP_INFO(
            this->get_logger(),
            "MCU E,0 - Emergency clear 확인"
        );


        if (
            phase_ == Phase::EMERGENCY &&
            mcu_state_ == 1
        )
        {
            phase_ =
                Phase::IDLE;

            has_pending_goal_ =
                false;


            RCLCPP_INFO(
                this->get_logger(),
                "S,1도 이미 확인됨 - Recovery 완료"
            );
        }
        else
        {
            RCLCPP_INFO(
                this->get_logger(),
                "STM S,1 대기"
            );
        }


        return;
    }
}
    // =========================================================
    // Nav Goal publish
    // =========================================================

    void send_nav_goal()
    {
        if (!has_pending_goal_)
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "전달할 pending goal 없음"
            );


            phase_ =
                Phase::IDLE;


            return;
        }


        if (mcu_state_ != 2)
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "Nav Goal 차단: MCU STATE != 2"
            );

            return;
        }


        if (emergency_)
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "Nav Goal 차단: Emergency"
            );

            return;
        }


        nav_goal_pub_->publish(
            pending_goal_
        );


        RCLCPP_INFO(
            this->get_logger(),
            "Nav Goal 승인: x=%.3f y=%.3f",
            pending_goal_.pose.position.x,
            pending_goal_.pose.position.y
        );


        has_pending_goal_ =
            false;


        phase_ =
            Phase::NAVIGATING;
    }


    // =========================================================
    // Nav cancel
    // =========================================================

    void request_nav_cancel()
    {
        std_msgs::msg::Bool
            msg;


        msg.data =
            true;


        nav_cancel_pub_->publish(
            msg
        );


        RCLCPP_WARN(
            this->get_logger(),
            "/nav/cancel = true"
        );
    }


    // =========================================================
    // Nav Finished
    // =========================================================

    void nav_finished_callback(
        const std_msgs::msg::Bool::SharedPtr msg
    )
    {
        if (!msg->data)
        {
            return;
        }


        // -----------------------------------------------------
        // Emergency 상태에서는
        // cancel 결과가 들어와도 절대 M1 송신하지 않음.
        // -----------------------------------------------------

        if (phase_ == Phase::EMERGENCY)
        {
            RCLCPP_INFO(
                this->get_logger(),
                "Emergency 중 Nav 종료 결과 수신 - 무시"
            );

            return;
        }


        if (phase_ != Phase::NAVIGATING)
        {
            return;
        }


        RCLCPP_INFO(
            this->get_logger(),
            "Navigation 종료 감지"
        );


        // -----------------------------------------------------
        // 혹시 Emergency callback과 timing이 겹친 경우
        // -----------------------------------------------------

        if (emergency_)
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Emergency 상태 - ADMITTANCE 요청 생략"
            );


            phase_ =
                Phase::EMERGENCY;


            return;
        }


        // -----------------------------------------------------
        // 이미 STATE1
        // -----------------------------------------------------

        if (mcu_state_ == 1)
        {
            phase_ =
                Phase::IDLE;


            RCLCPP_INFO(
                this->get_logger(),
                "MCU 이미 STATE1"
            );


            return;
        }


        // -----------------------------------------------------
        // 정상 Navigation 종료
        //
        // STATE2 -> M1 -> S1
        // -----------------------------------------------------

        request_mode(
    1
);

phase_ =
    Phase::WAIT_ADMITTANCE;

admittance_retry_count_ =
    0;

admittance_retry_exhausted_ =
    false;

mode_request_time_ =
    this->now();


        RCLCPP_INFO(
            this->get_logger(),
            "ADMITTANCE 복귀 요청 - MCU S,1 대기"
        );
    }


    // =========================================================
    // Mode request
    // =========================================================

    void request_mode(
        int mode
    )
    {
        std_msgs::msg::Int32
            msg;


        msg.data =
            mode;


        mode_cmd_pub_->publish(
            msg
        );


        RCLCPP_INFO(
            this->get_logger(),
            "/mcu/mode_cmd publish: %d",
            mode
        );
    }


    // =========================================================
    // Timer
    //
    // AUTO transition timeout만 사용
    // =========================================================

    void timer_callback()
{
    if (emergency_)
    {
        return;
    }


    // =====================================================
    // WAIT_AUTO
    // =====================================================

    if (phase_ == Phase::WAIT_AUTO)
    {
        const double elapsed =
            (
                this->now() -
                mode_request_time_
            ).seconds();


        if (elapsed < 2.0)
        {
            return;
        }


        RCLCPP_ERROR(
            this->get_logger(),
            "MCU AUTO 전환 Timeout - Pending Nav Goal 취소"
        );


        has_pending_goal_ =
            false;


        request_mode(
            1
        );


        phase_ =
            Phase::WAIT_ADMITTANCE;


        admittance_retry_count_ =
            0;

        admittance_retry_exhausted_ =
            false;


        mode_request_time_ =
            this->now();


        return;
    }


    // =====================================================
    // WAIT_ADMITTANCE
    //
    // M1 -> S1 timeout / retry
    // =====================================================

    if (phase_ == Phase::WAIT_ADMITTANCE)
    {
        if (admittance_retry_exhausted_)
        {
            return;
        }


        const double elapsed =
            (
                this->now() -
                mode_request_time_
            ).seconds();


        if (elapsed < 1.0)
        {
            return;
        }


        constexpr int MAX_ADMITTANCE_RETRIES =
            3;


        if (
            admittance_retry_count_ >=
            MAX_ADMITTANCE_RETRIES
        )
        {
            admittance_retry_exhausted_ =
                true;


            RCLCPP_ERROR(
                this->get_logger(),
                "========================================"
            );

            RCLCPP_ERROR(
                this->get_logger(),
                "MCU ADMITTANCE 전환 실패"
            );

            RCLCPP_ERROR(
                this->get_logger(),
                "M1 retry %d회 후에도 S1 없음",
                MAX_ADMITTANCE_RETRIES
            );

            RCLCPP_ERROR(
                this->get_logger(),
                "WAIT_ADMITTANCE 유지 - 새 목적지 차단"
            );

            RCLCPP_ERROR(
                this->get_logger(),
                "========================================"
            );


            return;
        }


        ++admittance_retry_count_;


        RCLCPP_WARN(
            this->get_logger(),
            "MCU S1 Timeout - M1 재전송 (%d/%d)",
            admittance_retry_count_,
            MAX_ADMITTANCE_RETRIES
        );


        request_mode(
            1
        );


        mode_request_time_ =
            this->now();


        return;
    }
}


    // =========================================================
    // ROS Interfaces
    // =========================================================

    rclcpp::Subscription<
        geometry_msgs::msg::PoseStamped
    >::SharedPtr destination_sub_;


    rclcpp::Subscription<
        std_msgs::msg::Int32
    >::SharedPtr state_sub_;


    rclcpp::Subscription<
        std_msgs::msg::Bool
    >::SharedPtr docked_sub_;


    rclcpp::Subscription<
        std_msgs::msg::Bool
    >::SharedPtr emergency_sub_;


    rclcpp::Subscription<
        std_msgs::msg::Bool
    >::SharedPtr nav_finished_sub_;


    rclcpp::Publisher<
        std_msgs::msg::Int32
    >::SharedPtr mode_cmd_pub_;


    rclcpp::Publisher<
        geometry_msgs::msg::PoseStamped
    >::SharedPtr nav_goal_pub_;


    rclcpp::Publisher<
        std_msgs::msg::Bool
    >::SharedPtr nav_cancel_pub_;


    rclcpp::TimerBase::SharedPtr
        timer_;


    // =========================================================
    // State
    // =========================================================

    int mcu_state_;

    bool docked_;

    bool emergency_;

    bool has_pending_goal_;
    
    int admittance_retry_count_;

    bool admittance_retry_exhausted_;


    geometry_msgs::msg::PoseStamped
        pending_goal_;


    Phase phase_;


    rclcpp::Time
        mode_request_time_;
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
            ModeManager
        >();


    rclcpp::spin(
        node
    );


    rclcpp::shutdown();


    return 0;
}
