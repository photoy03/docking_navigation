cmake_minimum_required(VERSION 3.10)
project(robot1_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# =====================================================================
# 🌟 완벽한 해법: 모든 소스를 가져온 뒤, 윈도우/맥 전용 파일만 깔끔하게 제거!
# 1. src 폴더 안의 모든 cpp 파일을 일단 다 가져옵니다.
file(GLOB CSERIALPORT_SOURCES "third_party/CSerialPort/src/*.cpp")

# 2. 정규식(REGEX)을 이용해 윈도우(Win)와 맥(Mac) 전용 파일을 리스트에서 삭제합니다.
# 이렇게 하면 실수로 필수 코드를 누락할 일이 0%가 됩니다.
list(FILTER CSERIALPORT_SOURCES EXCLUDE REGEX ".*WinBase\\.cpp|.*MacBase\\.cpp")
# =====================================================================

# 1. Odom Publisher
add_executable(odom_publisher src/odom_publisher.cpp)
ament_target_dependencies(odom_publisher rclcpp nav_msgs sensor_msgs std_msgs geometry_msgs tf2 tf2_ros)

# 2. MCU Bridge
# 🌟 필터링이 완료된 완벽한 소스 리스트(${CSERIALPORT_SOURCES})를 함께 빌드!
add_executable(mcu_bridge src/mcu_bridge.cpp ${CSERIALPORT_SOURCES})

# 헤더 경로 알려주기
target_include_directories(mcu_bridge PUBLIC third_party/CSerialPort/include)

# ROS2 의존성 추가
ament_target_dependencies(mcu_bridge rclcpp std_msgs geometry_msgs)

# 🌟 중요: 리눅스 시스템 스레드 라이브러리(pthread) 연결 (CSerialPort 필수 요구사항)
target_link_libraries(mcu_bridge pthread)

install(TARGETS
  odom_publisher
  mcu_bridge
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
