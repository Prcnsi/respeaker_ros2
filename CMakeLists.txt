find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AudioDataWithDOA.msg"
  DEPENDENCIES std_msgs  # 메시지가 std_msgs/Header 및 std_msgs/Int32를 사용
)
