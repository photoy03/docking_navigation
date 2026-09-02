import cv2
import time
import math
import numpy as np
import torch

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
)

from std_msgs.msg import Bool
from ultralytics import YOLO


# ============================================================
# Default Settings
# 나중에 navigation_launch.py에서 ROS parameter로 변경 가능
# ============================================================

DEFAULT_CAMERA_PATH = (
    "/dev/v4l/by-id/"
    "usb-046d_HD_Pro_Webcam_C920_6876141F-video-index0"
)

DEFAULT_MODEL_PATH = "/home/jet/yolo11n-pose.pt"


class FallDetectionNode(Node):

    def __init__(self):
        super().__init__("fall_detection_node")

        # ====================================================
        # ROS Parameters
        # ====================================================

        self.declare_parameter(
            "camera_path",
            DEFAULT_CAMERA_PATH
        )

        self.declare_parameter(
            "model_path",
            DEFAULT_MODEL_PATH
        )

        self.declare_parameter(
            "keypoint_conf_threshold",
            0.40
        )

        self.declare_parameter(
            "bbox_ratio_threshold",
            1.20
        )

        self.declare_parameter(
            "center_y_threshold",
            0.52
        )

        self.declare_parameter(
            "head_y_threshold",
            0.28
        )

        self.declare_parameter(
            "torso_angle_threshold",
            45.0
        )

        self.declare_parameter(
            "fall_confirm_time",
            1.5
        )

        self.declare_parameter(
            "image_size",
            640
        )


        # ====================================================
        # Parameter Load
        # ====================================================

        self.camera_path = (
            self.get_parameter(
                "camera_path"
            ).value
        )

        self.model_path = (
            self.get_parameter(
                "model_path"
            ).value
        )

        self.kp_conf = float(
            self.get_parameter(
                "keypoint_conf_threshold"
            ).value
        )

        self.bbox_threshold = float(
            self.get_parameter(
                "bbox_ratio_threshold"
            ).value
        )

        self.center_threshold = float(
            self.get_parameter(
                "center_y_threshold"
            ).value
        )

        self.head_threshold = float(
            self.get_parameter(
                "head_y_threshold"
            ).value
        )

        self.torso_threshold = float(
            self.get_parameter(
                "torso_angle_threshold"
            ).value
        )

        self.confirm_time = float(
            self.get_parameter(
                "fall_confirm_time"
            ).value
        )

        self.image_size = int(
            self.get_parameter(
                "image_size"
            ).value
        )


        # ====================================================
        # QoS
        #
        # /fall/detected
        # /mcu/emergency
        #
        # 상태값이므로 마지막 상태를 새 subscriber도
        # 받을 수 있도록 TRANSIENT_LOCAL 사용
        # ====================================================

        state_qos = QoSProfile(
            depth=1
        )

        state_qos.reliability = (
            ReliabilityPolicy.RELIABLE
        )

        state_qos.durability = (
            DurabilityPolicy.TRANSIENT_LOCAL
        )


        # ====================================================
        # ROS Publishers
        # ====================================================

        # 현재 카메라에서 판단한 낙상 상태
        self.fall_pub = self.create_publisher(
            Bool,
            "/fall/detected",
            state_qos
        )

        # 실제 Emergency 요청
        #
        # FALL 최초 확정 시 True 1회만 전송
        self.emergency_pub = self.create_publisher(
            Bool,
            "/mcu/emergency_cmd",
            10
        )


        # ====================================================
        # ROS Subscribers
        # ====================================================

        # 실제 MCU Emergency 상태
        #
        # mcu_bridge가 STM32 상태를 반영하여 publish
        self.emergency_sub = self.create_subscription(
            Bool,
            "/mcu/emergency",
            self.emergency_callback,
            state_qos
        )


        # ====================================================
        # Fall Detection State
        # ====================================================

        # 낙상 후보가 시작된 시간
        self.candidate_start_time = None

        # 현재 카메라가 FALL이라고 판단 중인지
        self.fall_detected = False

        # /fall/detected 마지막 publish 값
        self.last_published_fall = None


        # ====================================================
        # Emergency State
        # ====================================================

        # 이번 Emergency cycle에서
        # 이미 Emergency 명령을 보냈는지
        self.emergency_request_sent = False

        # MCU가 실제 Emergency 상태인지
        self.mcu_emergency = False

        # MCU Emergency True를 실제 확인한 적이 있는지
        #
        # 시작할 때 전달되는 false 때문에
        # emergency_request_sent가 잘못 초기화되는 것을 방지
        self.mcu_emergency_confirmed = False


        # ====================================================
        # Start Log
        # ====================================================

        self.get_logger().info(
            "========================================"
        )

        self.get_logger().info(
            "Fall Detection Node START"
        )

        self.get_logger().info(
            f"Camera: {self.camera_path}"
        )

        self.get_logger().info(
            f"Model: {self.model_path}"
        )

        self.get_logger().info(
            f"Fall confirm time: {self.confirm_time:.2f} sec"
        )


        # ====================================================
        # GPU
        # ====================================================

        self.cuda_available = (
            torch.cuda.is_available()
        )

        self.get_logger().info(
            f"CUDA available: {self.cuda_available}"
        )

        if self.cuda_available:
            self.device = 0

            self.get_logger().info(
                f"GPU: {torch.cuda.get_device_name(0)}"
            )

        else:
            self.device = "cpu"

            self.get_logger().warning(
                "CUDA unavailable - CPU inference"
            )


        # ====================================================
        # YOLO Pose
        # ====================================================

        self.get_logger().info(
            "Loading YOLO Pose model..."
        )

        self.model = YOLO(
            self.model_path
        )

        self.get_logger().info(
            "YOLO Pose model loaded"
        )


        # ====================================================
        # Camera
        # ====================================================

        self.cap = cv2.VideoCapture(
            self.camera_path,
            cv2.CAP_V4L2
        )

        self.cap.set(
            cv2.CAP_PROP_FOURCC,
            cv2.VideoWriter_fourcc(
                *"MJPG"
            )
        )

        self.cap.set(
            cv2.CAP_PROP_FRAME_WIDTH,
            1280
        )

        self.cap.set(
            cv2.CAP_PROP_FRAME_HEIGHT,
            720
        )

        self.cap.set(
            cv2.CAP_PROP_FPS,
            30
        )


        if not self.cap.isOpened():
            self.get_logger().fatal(
                "C920 camera open failed"
            )

            raise RuntimeError(
                "Camera open failed"
            )


        self.frame_h = int(
            self.cap.get(
                cv2.CAP_PROP_FRAME_HEIGHT
            )
        )

        self.frame_w = int(
            self.cap.get(
                cv2.CAP_PROP_FRAME_WIDTH
            )
        )


        self.get_logger().info(
            f"Camera resolution: "
            f"{self.frame_w} x {self.frame_h}"
        )


        # 카메라 시작 직후 불안정한 frame 버림
        for _ in range(10):
            self.cap.read()


        # 시작 시 정상 상태 publish
        self.publish_fall_state(
            False,
            force=True
        )


        # ====================================================
        # Main Timer
        #
        # 실제 주기는 YOLO inference 속도에 의해 결정됨
        # ====================================================

        self.timer = self.create_timer(
            0.01,
            self.process_frame
        )


        self.get_logger().info(
            "Fall Detection ready"
        )

        self.get_logger().info(
            "========================================"
        )


    # ========================================================
    # MCU Emergency Feedback
    # ========================================================

    def emergency_callback(self, msg):

        new_state = bool(
            msg.data
        )

        previous_state = (
            self.mcu_emergency
        )

        self.mcu_emergency = (
            new_state
        )


        # ----------------------------------------------------
        # MCU Emergency 실제 확인
        # ----------------------------------------------------

        if (
            new_state
            and not previous_state
        ):
            self.mcu_emergency_confirmed = True

            self.get_logger().error(
                "MCU Emergency latch CONFIRMED"
            )


        # ----------------------------------------------------
        # MCU Emergency True → False
        #
        # STM32 Reset / 재부팅 이후에만 발생한다고 가정
        # ----------------------------------------------------

        elif (
            not new_state
            and previous_state
            and self.mcu_emergency_confirmed
        ):

            self.get_logger().info(
                "MCU Emergency released "
                "(STM32 reset / reboot detected)"
            )

            # 새로운 Emergency cycle 허용
            self.emergency_request_sent = False

            self.mcu_emergency_confirmed = False


            # 만약 Reset했는데도 사람이 여전히
            # FALL 상태라면 즉시 다시 Emergency 요청
            if self.fall_detected:

                self.get_logger().warning(
                    "Person is still FALL after MCU reset"
                )

                self.send_emergency_request()


    # ========================================================
    # Torso Angle
    #
    # 0 deg  = vertical
    # 90 deg = horizontal
    # ========================================================

    def calculate_torso_angle(
        self,
        keypoints
    ):

        required = [
            5,      # left shoulder
            6,      # right shoulder
            11,     # left hip
            12      # right hip
        ]


        # 네 점 중 하나라도 confidence 부족하면
        # torso angle 사용하지 않음
        for idx in required:

            if (
                keypoints[idx][2]
                < self.kp_conf
            ):
                return None


        left_shoulder = (
            keypoints[5][:2]
        )

        right_shoulder = (
            keypoints[6][:2]
        )

        left_hip = (
            keypoints[11][:2]
        )

        right_hip = (
            keypoints[12][:2]
        )


        shoulder_mid = (
            left_shoulder
            + right_shoulder
        ) / 2.0


        hip_mid = (
            left_hip
            + right_hip
        ) / 2.0


        dx = (
            hip_mid[0]
            - shoulder_mid[0]
        )

        dy = (
            hip_mid[1]
            - shoulder_mid[1]
        )


        angle = math.degrees(
            math.atan2(
                abs(dx),
                abs(dy) + 1e-6
            )
        )


        return float(
            angle
        )


    # ========================================================
    # Main Frame Processing
    # ========================================================

    def process_frame(self):

        ret, frame = (
            self.cap.read()
        )


        if not ret:
            self.get_logger().warning(
                "Camera frame read failed"
            )

            return


        # ====================================================
        # YOLO Pose Inference
        # ====================================================

        results = self.model(
            frame,
            device=self.device,
            imgsz=self.image_size,
            verbose=False
        )


        result = results[0]


        # ====================================================
        # Person 없음
        # ====================================================

        if (
            result.boxes is None
            or result.keypoints is None
            or len(result.boxes) == 0
        ):

            self.candidate_start_time = None

            self.set_fall_state(
                False
            )

            return


        # ====================================================
        # Person 선택
        #
        # 현재 최종 버전:
        # 화면에서 가장 큰 사람 1명 사용
        #
        # 추후 필요하면 YOLO Tracking으로 확장 가능
        # ====================================================

        boxes = (
            result.boxes.xyxy
            .detach()
            .cpu()
            .numpy()
        )


        keypoints_all = (
            result.keypoints.data
            .detach()
            .cpu()
            .numpy()
        )


        areas = (
            (boxes[:, 2] - boxes[:, 0])
            *
            (boxes[:, 3] - boxes[:, 1])
        )


        idx = int(
            np.argmax(areas)
        )


        x1, y1, x2, y2 = (
            boxes[idx]
        )


        keypoints = (
            keypoints_all[idx]
        )


        # ====================================================
        # Feature 1
        # Bounding Box Ratio
        # ====================================================

        bbox_width = (
            x2 - x1
        )

        bbox_height = (
            y2 - y1
        )


        if bbox_height > 0:

            bbox_ratio = (
                bbox_width
                / bbox_height
            )

        else:

            bbox_ratio = 0.0


        # ====================================================
        # Feature 2
        # Bounding Box Center Y
        # ====================================================

        center_y_norm = (
            ((y1 + y2) / 2.0)
            / self.frame_h
        )


        # ====================================================
        # Feature 3
        # Head Y
        #
        # COCO keypoint 0 = nose
        # ====================================================

        head_y_norm = None


        if (
            keypoints[0][2]
            >= self.kp_conf
        ):

            head_y_norm = (
                keypoints[0][1]
                / self.frame_h
            )


        # ====================================================
        # Feature 4
        # Torso Angle
        # ====================================================

        torso_angle = (
            self.calculate_torso_angle(
                keypoints
            )
        )


        # ====================================================
        # Threshold Conditions
        # ====================================================

        bbox_cond = (
            bbox_ratio
            > self.bbox_threshold
        )


        center_cond = (
            center_y_norm
            > self.center_threshold
        )


        head_cond = (
            head_y_norm is not None
            and
            head_y_norm
            > self.head_threshold
        )


        torso_cond = (
            torso_angle is not None
            and
            torso_angle
            > self.torso_threshold
        )


        # ====================================================
        # Fall Candidate Logic
        #
        # Torso 사용 가능:
        #   Torso horizontal
        #          AND
        #   BBox / Center / Head 중 하나
        #
        # Torso 사용 불가:
        #   BBox AND Center AND Head
        # ====================================================

        if torso_angle is not None:

            fall_candidate = (
                torso_cond
                and
                (
                    bbox_cond
                    or center_cond
                    or head_cond
                )
            )

        else:

            fall_candidate = (
                bbox_cond
                and center_cond
                and head_cond
            )


        # ====================================================
        # Temporal Confirmation
        # ====================================================

        now = time.monotonic()


        if fall_candidate:

            # 후보 시작
            if (
                self.candidate_start_time
                is None
            ):

                self.candidate_start_time = (
                    now
                )


            duration = (
                now
                - self.candidate_start_time
            )


            # 설정 시간 이상 FALL 상태 유지
            if (
                duration
                >= self.confirm_time
            ):

                self.set_fall_state(
                    True
                )


        else:

            self.candidate_start_time = None

            # -----------------------------------------------
            # 사람이 일어난 경우
            #
            # 카메라 FALL 상태만 False로 변경
            #
            # Emergency는 절대 여기서 해제하지 않는다.
            # -----------------------------------------------

            self.set_fall_state(
                False
            )


    # ========================================================
    # Fall State Handler
    # ========================================================

    def set_fall_state(
        self,
        state
    ):

        state = bool(
            state
        )


        # ====================================================
        # NORMAL → FALL
        # ====================================================

        if (
            state
            and not self.fall_detected
        ):

            self.fall_detected = True


            self.get_logger().error(
                "========================================"
            )

            self.get_logger().error(
                "FALL DETECTED"
            )

            self.get_logger().error(
                "========================================"
            )


            # Emergency 요청
            self.send_emergency_request()


        # ====================================================
        # FALL → NORMAL
        # ====================================================

        elif (
            not state
            and self.fall_detected
        ):

            self.fall_detected = False


            self.get_logger().info(
                "Fall state returned to NORMAL"
            )

            self.get_logger().warning(
                "Emergency latch remains active"
            )


        # 현재 Fall 상태 publish
        self.publish_fall_state(
            state
        )


    # ========================================================
    # Emergency Request
    # ========================================================

    def send_emergency_request(self):

        # 이미 이번 Emergency cycle에서
        # 요청을 보냈으면 다시 전송하지 않음
        if self.emergency_request_sent:

            return


        # MCU가 이미 Emergency 상태라면
        # 추가 명령 불필요
        if self.mcu_emergency:

            self.emergency_request_sent = True

            return


        emergency_msg = Bool()

        emergency_msg.data = True


        self.emergency_pub.publish(
            emergency_msg
        )


        # publish 직후 바로 latch
        #
        # 다음 YOLO frame에서 같은 명령이
        # 반복 발행되는 것을 방지
        self.emergency_request_sent = True


        self.get_logger().error(
            "/mcu/emergency_cmd = TRUE"
        )

        self.get_logger().error(
            "Emergency request sent ONCE"
        )


    # ========================================================
    # Fall State Publisher
    # ========================================================

    def publish_fall_state(
        self,
        state,
        force=False
    ):

        state = bool(
            state
        )


        # 상태가 바뀐 경우에만 publish
        if (
            not force
            and
            self.last_published_fall
            == state
        ):

            return


        msg = Bool()

        msg.data = state


        self.fall_pub.publish(
            msg
        )


        self.last_published_fall = (
            state
        )


    # ========================================================
    # Shutdown
    # ========================================================

    def destroy_node(self):

        self.get_logger().info(
            "Fall Detection Node shutdown"
        )


        if (
            hasattr(self, "cap")
            and self.cap is not None
            and self.cap.isOpened()
        ):

            self.cap.release()


        super().destroy_node()


# ============================================================
# Main
# ============================================================

def main(args=None):

    rclpy.init(
        args=args
    )


    node = None


    try:

        node = FallDetectionNode()

        rclpy.spin(
            node
        )


    except KeyboardInterrupt:

        pass


    finally:

        if node is not None:

            node.destroy_node()


        if rclpy.ok():

            rclpy.shutdown()


if __name__ == "__main__":

    main()
