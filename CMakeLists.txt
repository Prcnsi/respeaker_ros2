cmake_minimum_required(VERSION 3.22)

# 프로젝트 이름 설정 (예시: respeaker_ros2)
project(respeaker_ros2)

# 필수 패키지 찾기
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# 메시지 파일 처리
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AudioDataWithDOA.msg"
  DEPENDENCIES std_msgs
)
