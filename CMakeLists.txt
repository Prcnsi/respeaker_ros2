# CMake 최소 요구 사항
cmake_minimum_required(VERSION 3.22)

# 프로젝트 이름 정의
project(respeaker_ros2)

# 필요한 패키지 찾기
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# 메시지 파일 정의
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AudioDataWithDOA.msg"
  DEPENDENCIES std_msgs
)

# 의존성 추가
ament_package()

