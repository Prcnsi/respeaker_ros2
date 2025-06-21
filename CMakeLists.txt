cmake_minimum_required(VERSION 3.5)
project(respeaker_ros2)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)  # 예시: 추가적인 메시지 의존성

# 메시지 파일을 생성하고 빌드하기 위한 설정
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AudioDataWithDOA.msg"  # AudioDataWithDOA.msg 파일을 사용
  DEPENDENCIES std_msgs  # std_msgs를 의존성에 추가
)

# Python 패키지 빌드를 위한 설정 (Python 파일을 사용하고 있으면)
ament_python_install_package()

# 필요한 다른 설정들...
ament_package()
