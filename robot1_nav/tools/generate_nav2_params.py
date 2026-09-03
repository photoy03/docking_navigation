#!/usr/bin/env python3

from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory


# ==========================================================
# Paths
# ==========================================================

NAV2_BRINGUP_SHARE = Path(
    get_package_share_directory("nav2_bringup")
)

NAV2_SMAC_SHARE = Path(
    get_package_share_directory("nav2_smac_planner")
)

SRC = (
    NAV2_BRINGUP_SHARE
    / "params"
    / "nav2_params.yaml"
)

DST = (
    Path.home()
    / "ros2_ws"
    / "src"
    / "robot1_nav"
    / "config"
    / "nav2_params.yaml"
)


# ==========================================================
# Smac State Lattice primitive
#
# ROS2 Humble
# Global Costmap resolution = 0.05 m
# Differential Drive
# ==========================================================

LATTICE_CANDIDATES = [
    NAV2_SMAC_SHARE
    / "sample_primitives"
    / "5cm_resolution"
    / "0.5m_turning_radius"
    / "diff"
    / "output.json",

    NAV2_SMAC_SHARE
    / "sample_primitives"
    / "5cm_resolution"
    / "1m_turning_radius"
    / "diff"
    / "output.json",
]

LATTICE_FILE = next(
    (
        path
        for path in LATTICE_CANDIDATES
        if path.exists()
    ),
    None,
)

if LATTICE_FILE is None:
    raise FileNotFoundError(
        "Smac State Lattice differential primitive를 찾지 못했습니다.\n"
        "다음 명령으로 설치된 primitive 경로를 확인하세요:\n"
        "find $(ros2 pkg prefix nav2_smac_planner)"
        "/share/nav2_smac_planner "
        "-name output.json -print"
    )


print("Nav2 source params :", SRC)
print("Robot1 output params:", DST)
print("State Lattice file :", LATTICE_FILE)


# ==========================================================
# Load base Nav2 parameters
# ==========================================================

with open(SRC, "r", encoding="utf-8") as f:
    data = yaml.safe_load(f)


# ==========================================================
# 전체 use_sim_time = false
# ==========================================================

def recursive_sim_time_false(obj):
    if isinstance(obj, dict):
        for key, value in obj.items():
            if key == "use_sim_time":
                obj[key] = False
            else:
                recursive_sim_time_false(value)

    elif isinstance(obj, list):
        for value in obj:
            recursive_sim_time_false(value)


recursive_sim_time_false(data)


# ==========================================================
# Robot Footprint
#
# robot1_base:
#   drive axle midpoint floor projection
#
# +X forward
# +Y left
# ==========================================================

ROBOT_FOOTPRINT = (
    "[[0.29, -0.65], "
    "[0.29, 0.65], "
    "[-0.95, 0.65], "
    "[-0.95, -0.65]]"
)


# ==========================================================
# AMCL
#
# 목적:
#   - LiDAR 기반 localization 신뢰성 유지
#   - 지나치게 많은 particle/beam 처리로 인한
#     CPU 및 TF queue 밀림 완화
#
# 기존:
#   max_beams      = 100
#   min_particles  = 500
#   max_particles  = 2000
#   update_min_d   = 0.01
#   update_min_a   = 0.02
#
# 변경:
#   max_beams      = 60
#   min_particles  = 400
#   max_particles  = 1500
#   update_min_d   = 0.03
#   update_min_a   = 0.05
# ==========================================================

amcl = data["amcl"]["ros__parameters"]

amcl["base_frame_id"] = "robot1_base"
amcl["odom_frame_id"] = "robot1_odom"
amcl["global_frame_id"] = "map"
amcl["scan_topic"] = "/scan_filtered"

amcl["robot_model_type"] = (
    "nav2_amcl::DifferentialMotionModel"
)


# ----------------------------------------------------------
# Odom Motion Model
# ----------------------------------------------------------

amcl["alpha1"] = 0.25
amcl["alpha2"] = 0.25
amcl["alpha3"] = 0.25
amcl["alpha4"] = 0.25
amcl["alpha5"] = 0.20


# ----------------------------------------------------------
# Laser Model
# ----------------------------------------------------------

amcl["laser_model_type"] = "likelihood_field_prob"

amcl["z_hit"] = 0.80
amcl["z_rand"] = 0.20

amcl["sigma_hit"] = 0.20

amcl["laser_likelihood_max_dist"] = 2.0


# ----------------------------------------------------------
# Beam Skip
# ----------------------------------------------------------

amcl["do_beamskip"] = True

amcl["beam_skip_distance"] = 0.5
amcl["beam_skip_threshold"] = 0.3
amcl["beam_skip_error_threshold"] = 0.9


# ----------------------------------------------------------
# Particle / Scan processing
#
# TF/AMCL queue 밀림을 줄이기 위해
# 기존보다 처리량을 약간 완화
# ----------------------------------------------------------

amcl["max_beams"] = 60

amcl["min_particles"] = 400
amcl["max_particles"] = 1500

amcl["resample_interval"] = 1


# ----------------------------------------------------------
# Measurement update threshold
#
# 0.15 m/s 저속 플랫폼에서:
#   0.03 m ≒ 약 0.2초 이동
#
# LiDAR 10~11 Hz에 비해 지나치게 잦은
# AMCL 계산을 막으면서 localization은 유지
# ----------------------------------------------------------

amcl["update_min_d"] = 0.03
amcl["update_min_a"] = 0.05


# ----------------------------------------------------------
# TF
# ----------------------------------------------------------

# AMCL map -> robot1_odom TF tolerance.
# 기존 값 유지.
amcl["transform_tolerance"] = 1.0

amcl["tf_broadcast"] = True


# ----------------------------------------------------------
# LiDAR range
# ----------------------------------------------------------

amcl["laser_min_range"] = -1.0
amcl["laser_max_range"] = -1.0


# ----------------------------------------------------------
# Initial Pose
#
# 현재 docking start 위치 기준
# ----------------------------------------------------------

amcl["set_initial_pose"] = True

amcl["initial_pose"] = {
    "x": 0.0016854011919349432,
    "y": -0.00030811282340437174,
    "z": 0.0,
    "yaw": 0.00948,
}

amcl["always_reset_initial_pose"] = True


# ==========================================================
# BT Navigator
#
# 기존 10 ms = 최대 100 Hz tick
# 최근 시스템에서는 과도한 BT tick warning이 있었으므로
# 20 ms = 최대 50 Hz로 완화
#
# Controller가 20 Hz이므로 50 Hz BT는 충분
# ==========================================================

bt = data[
    "bt_navigator"
]["ros__parameters"]

bt["global_frame"] = "map"
bt["robot_base_frame"] = "robot1_base"
bt["odom_topic"] = "/robot1/odom"

# milliseconds
bt["bt_loop_duration"] = 20

# milliseconds
bt["default_server_timeout"] = 300

# milliseconds
bt["wait_for_service_timeout"] = 1000


# ==========================================================
# Controller Server
#
# Regulated Pure Pursuit
# ==========================================================

controller = data[
    "controller_server"
]["ros__parameters"]


# ----------------------------------------------------------
# Controller execution
# ----------------------------------------------------------

controller["controller_frequency"] = 20.0


# ----------------------------------------------------------
# 순간적인 Controller failure 허용
#
# TF jitter / temporary collision prediction 등이
# 한두 cycle 발생했다고 즉시 BT Recovery로 넘어가지 않음
# ----------------------------------------------------------

controller["failure_tolerance"] = 1.0


# ----------------------------------------------------------
# Progress Checker
#
# 기존:
#   0.05 m / 20 sec
#
# 변경:
#   0.04 m / 25 sec
#
# 대형 저속 플랫폼에서 recovery 과민 진입 완화
# ----------------------------------------------------------

controller["progress_checker"][
    "required_movement_radius"
] = 0.04

controller["progress_checker"][
    "movement_time_allowance"
] = 25.0


# ----------------------------------------------------------
# Goal Checker
# ----------------------------------------------------------

controller["general_goal_checker"][
    "xy_goal_tolerance"
] = 0.15

# 현재 실차 검증에서는 final heading 강제하지 않음
controller["general_goal_checker"][
    "yaw_goal_tolerance"
] = 3.14159


# ==========================================================
# Regulated Pure Pursuit
# ==========================================================

controller["FollowPath"] = {
    "plugin":
        "nav2_regulated_pure_pursuit_controller::"
        "RegulatedPurePursuitController",

    # ------------------------------------------------------
    # Linear velocity
    # ------------------------------------------------------

    "desired_linear_vel": 0.15,

    # ------------------------------------------------------
    # Lookahead
    #
    # 기존보다 먼 경로를 바라보게 해서
    # 좌우 조향 command oscillation 완화
    # ------------------------------------------------------

    "lookahead_dist": 0.60,
    "min_lookahead_dist": 0.45,
    "max_lookahead_dist": 0.80,

    "lookahead_time": 3.0,

    "use_velocity_scaled_lookahead_dist": True,

    # ------------------------------------------------------
    # Goal approach
    # ------------------------------------------------------

    "min_approach_linear_velocity": 0.05,

    "approach_velocity_scaling_dist": 0.80,

    # ------------------------------------------------------
    # Collision Detection
    #
    # Collision 기능 자체는 절대 끄지 않음.
    #
    # 기존 1.0 sec -> 0.7 sec
    # 지나치게 먼 예측으로 recovery 진입하는 현상 완화
    # ------------------------------------------------------

    "use_collision_detection": True,

    "max_allowed_time_to_collision_up_to_carrot": 0.70,

    # ------------------------------------------------------
    # Curvature based velocity regulation
    # ------------------------------------------------------

    "use_regulated_linear_velocity_scaling": True,

    "regulated_linear_scaling_min_radius": 1.0,

    "regulated_linear_scaling_min_speed": 0.05,

    # ------------------------------------------------------
    # Cost based velocity regulation
    #
    # 아직 OFF.
    #
    # inflation_cost_scaling_factor는 향후 활성화 대비
    # Local Costmap inflation factor 3.5와 일치시켜 둔다.
    # ------------------------------------------------------

    "use_cost_regulated_linear_velocity_scaling": False,

    "cost_scaling_dist": 0.55,

    "cost_scaling_gain": 1.0,

    "inflation_cost_scaling_factor": 3.5,

    # ------------------------------------------------------
    # Heading alignment
    #
    # 작은 heading 차이에서는 주행하며 보정하고
    # 큰 차이에서만 제자리 회전
    # ------------------------------------------------------

    "use_rotate_to_heading": True,

    "rotate_to_heading_min_angle": 0.75,

    "rotate_to_heading_angular_vel": 0.28,

    "max_angular_accel": 0.60,

    # ------------------------------------------------------
    # TF
    #
    # 짧은 TF jitter 허용
    # ------------------------------------------------------

    "transform_tolerance": 0.30,

    # ------------------------------------------------------
    # Path
    # ------------------------------------------------------

    "use_interpolation": True,
}


# ==========================================================
# Local Costmap
#
# 역할:
#   실제 주행 중 LiDAR obstacle / collision 판단
#
# Global planning clearance와 독립적으로 유지
# ==========================================================

local = data[
    "local_costmap"
]["local_costmap"]["ros__parameters"]

local["global_frame"] = "robot1_odom"
local["robot_base_frame"] = "robot1_base"

local["resolution"] = 0.05

local["rolling_window"] = True

local["width"] = 3
local["height"] = 3

local["update_frequency"] = 5.0
local["publish_frequency"] = 2.0

# ----------------------------------------------------------
# CPU / DDS optimization
# ----------------------------------------------------------

local["voxel_layer"]["publish_voxel_map"] = False

local["always_send_full_costmap"] = False

# ----------------------------------------------------------
# TF
#
# Nav2 Costmap 기본값도 0.3이지만
# baseline 명확화를 위해 명시
# ----------------------------------------------------------
local["voxel_layer"]["scan"]["observation_persistence"] = 0.0

local["transform_tolerance"] = 0.30


# ----------------------------------------------------------
# Footprint
# ----------------------------------------------------------

local["footprint"] = ROBOT_FOOTPRINT

local.pop(
    "robot_radius",
    None,
)


# ----------------------------------------------------------
# Local Inflation
#
# 이번 단계에서는 기존 값 유지
# ----------------------------------------------------------

local["inflation_layer"][
    "inflation_radius"
] = 0.55

local["inflation_layer"][
    "cost_scaling_factor"
] = 3.5


# ----------------------------------------------------------
# LiDAR
# ----------------------------------------------------------

local["voxel_layer"]["scan"][
    "topic"
] = "/scan_filtered"


# ----------------------------------------------------------
# Keepout
# ----------------------------------------------------------

local["filters"] = [
    "keepout_filter"
]

local["keepout_filter"] = {
    "plugin":
        "nav2_costmap_2d::KeepoutFilter",

    "enabled": True,

    "filter_info_topic":
        "/keepout_costmap_filter_info",
}


# ==========================================================
# Global Costmap
#
# 목표:
#   Global path가 벽 / Keepout 경계에 너무 붙는 현상 완화
#
# 기존:
#   inflation radius = 0.70
#   scaling factor   = 3.0
#
# 변경:
#   inflation radius = 0.85
#   scaling factor   = 2.5
#
# 더 넓고 완만한 cost gradient 생성
# ==========================================================

global_costmap = data[
    "global_costmap"
]["global_costmap"]["ros__parameters"]

global_costmap["global_frame"] = "map"

global_costmap[
    "robot_base_frame"
] = "robot1_base"

global_costmap["resolution"] = 0.05

global_costmap["update_frequency"] = 1.0
global_costmap["publish_frequency"] = 1.0

# ----------------------------------------------------------
# CPU / DDS optimization
# ----------------------------------------------------------

global_costmap["always_send_full_costmap"] = False

global_costmap["track_unknown_space"] = True


# ----------------------------------------------------------
# TF
# ----------------------------------------------------------
global_costmap["obstacle_layer"]["scan"]["observation_persistence"] = 0.0
global_costmap[
    "transform_tolerance"
] = 0.30


# ----------------------------------------------------------
# Footprint
# ----------------------------------------------------------

global_costmap[
    "footprint"
] = ROBOT_FOOTPRINT

global_costmap.pop(
    "robot_radius",
    None,
)


# ----------------------------------------------------------
# Global Inflation
# ----------------------------------------------------------

global_costmap["inflation_layer"][
    "inflation_radius"
] = 0.85

global_costmap["inflation_layer"][
    "cost_scaling_factor"
] = 2.5


# ----------------------------------------------------------
# LiDAR
# ----------------------------------------------------------

global_costmap[
    "obstacle_layer"
]["scan"]["topic"] = "/scan_filtered"


# ----------------------------------------------------------
# Keepout
# ----------------------------------------------------------

global_costmap["filters"] = [
    "keepout_filter"
]

global_costmap["keepout_filter"] = {
    "plugin":
        "nav2_costmap_2d::KeepoutFilter",

    "enabled": True,

    "filter_info_topic":
        "/keepout_costmap_filter_info",
}


# ==========================================================
# Planner Server
#
# Smac State Lattice
# ROS2 Humble
# ==========================================================

planner = data[
    "planner_server"
]["ros__parameters"]


# State Lattice는 NavFn보다 무거움.
planner[
    "expected_planner_frequency"
] = 1.0


planner["planner_plugins"] = [
    "GridBased"
]


planner["GridBased"] = {
    "plugin":
        "nav2_smac_planner/SmacPlannerLattice",

    # ------------------------------------------------------
    # Static map
    # ------------------------------------------------------

    "allow_unknown": False,

    "tolerance": 0.25,

    # ------------------------------------------------------
    # Planning limits
    # ------------------------------------------------------

    "max_iterations": 1000000,

    "max_on_approach_iterations": 1000,

    "max_planning_time": 5.0,

    # ------------------------------------------------------
    # Analytic expansion
    # ------------------------------------------------------

    "analytic_expansion_ratio": 3.5,

    "analytic_expansion_max_length": 3.0,

    # ------------------------------------------------------
    # Search penalties
    # ------------------------------------------------------

    "reverse_penalty": 2.0,

    "change_penalty": 0.05,

    "non_straight_penalty": 1.05,

    # 높은 cost 영역을 기존보다 강하게 회피
    # -> 복도 중앙 쪽 경로 선호
    "cost_penalty": 4.0,

    # 대형 비원형 플랫폼의 불필요한 회전 억제
    "rotation_penalty": 5.0,

    "retrospective_penalty": 0.015,

    # ------------------------------------------------------
    # Lattice
    # ------------------------------------------------------

    "lattice_filepath":
        str(LATTICE_FILE),

    "lookup_table_size": 20.0,

    # Global obstacle cost가 동적으로 바뀌므로
    # 우선 cache 비활성
    "cache_obstacle_heuristic": False,

    # 기본 global path는 후진 경로 생성하지 않음
    "allow_reverse_expansion": False,

    # ------------------------------------------------------
    # Path smoothing
    # ------------------------------------------------------

    "smooth_path": True,
}


# ==========================================================
# Behavior Server / Recovery
#
# 기존:
#   max rot = 0.25
#   min rot = 0.18
#   accel   = 0.40
#   ahead   = 2.0
#
# 변경:
#   max rot = 0.35
#   min rot = 0.22
#   accel   = 0.70
#   ahead   = 1.5
#
# 느린 제자리 회전 및 달달거림 완화
# ==========================================================

behavior = data[
    "behavior_server"
]["ros__parameters"]

behavior[
    "global_frame"
] = "robot1_odom"

behavior[
    "robot_base_frame"
] = "robot1_base"

behavior[
    "cycle_frequency"
] = 10.0


# ----------------------------------------------------------
# Recovery collision simulation
# ----------------------------------------------------------

behavior[
    "simulate_ahead_time"
] = 1.5


# ----------------------------------------------------------
# Recovery rotation
# ----------------------------------------------------------

behavior[
    "max_rotational_vel"
] = 0.35

behavior[
    "min_rotational_vel"
] = 0.22

behavior[
    "rotational_acc_lim"
] = 0.70


# ----------------------------------------------------------
# TF
# ----------------------------------------------------------

behavior[
    "transform_tolerance"
] = 0.30


# ==========================================================
# Velocity Smoother
# ==========================================================

velocity = data[
    "velocity_smoother"
]["ros__parameters"]


velocity[
    "smoothing_frequency"
] = 20.0

velocity[
    "feedback"
] = "OPEN_LOOP"


# ----------------------------------------------------------
# Velocity limits
# ----------------------------------------------------------

velocity["max_velocity"] = [
    0.20,
    0.0,
    0.60,
]

velocity["min_velocity"] = [
    -0.10,
    0.0,
    -0.60,
]


# ----------------------------------------------------------
# Acceleration limits
# ----------------------------------------------------------

velocity["max_accel"] = [
    0.50,
    0.0,
    1.0,
]

velocity["max_decel"] = [
    -0.50,
    0.0,
    -1.0,
]


# ----------------------------------------------------------
# Odom
# ----------------------------------------------------------

velocity[
    "odom_topic"
] = "/robot1/odom"


# STM32 cmd timeout = 300 ms
# ROS velocity timeout은 이보다 짧게 유지
velocity[
    "velocity_timeout"
] = 0.20


# ==========================================================
# Map Server
#
# 실제 my_map.yaml은 navigation_launch.py에서 주입
# ==========================================================

data[
    "map_server"
]["ros__parameters"][
    "yaml_filename"
] = ""


# ==========================================================
# Keepout Zone
#
# 실제 my_map_keepout.yaml 경로는
# navigation_launch.py에서 주입
# ==========================================================

data[
    "keepout_filter_mask_server"
] = {
    "ros__parameters": {
        "use_sim_time": False,

        "topic_name":
            "/keepout_filter_mask",
    }
}


data[
    "keepout_costmap_filter_info_server"
] = {
    "ros__parameters": {
        "use_sim_time": False,

        "type": 0,

        "filter_info_topic":
            "/keepout_costmap_filter_info",

        "mask_topic":
            "/keepout_filter_mask",

        "base": 0.0,

        "multiplier": 1.0,
    }
}


# ==========================================================
# Save
# ==========================================================

DST.parent.mkdir(
    parents=True,
    exist_ok=True,
)

with open(
    DST,
    "w",
    encoding="utf-8",
) as f:
    yaml.safe_dump(
        data,
        f,
        sort_keys=False,
        default_flow_style=False,
    )


# ==========================================================
# Summary
# ==========================================================

print()
print(
    "=============================================="
)
print(
    "Robot1 Nav2 parameters generated successfully"
)
print(
    "=============================================="
)

print("Output :", DST)

print(
    "Planner: SmacPlannerLattice"
)

print(
    "Lattice:",
    LATTICE_FILE,
)

print(
    "AMCL   : beams=60, particles=400~1500, "
    "update=0.03m/0.05rad"
)

print(
    "Global : inflation=0.85, scaling=2.5, "
    "Smac cost_penalty=4.0"
)

print(
    "Local  : inflation=0.55, scaling=3.5"
)

print(
    "RPP    : lookahead=0.45~0.80, "
    "collision=0.70s"
)

print(
    "RPP    : cost regulation disabled"
)

print(
    "Recovery: rot=0.22~0.35 rad/s, "
    "accel=0.70"
)

print(
    "TF     : controller/behavior/costmap=0.30s, "
    "AMCL=1.00s"
)

print(
    "BT     : loop=20ms, server_timeout=50ms"
)

print(
    "Keepout: enabled on local/global costmaps"
)
