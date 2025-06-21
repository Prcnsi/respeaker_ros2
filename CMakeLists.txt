cmake_minimum_required(VERSION 3.22)
project(respeaker_ros2)

find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AudioDataWithDOA.msg"
  DEPENDENCIES std_msgs
)
ament_package()

