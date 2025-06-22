cmake_minimum_required(VERSION 3.22)
project(respeaker_ros2)

find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS src/respeaker_ros2/audio_with_doa_publisher.py
        DESTINATION lib/${PROJECT_NAME}
        RENAME audio_with_doa_node )
        
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AudioDataWithDOA.msg"
  DEPENDENCIES std_msgs
)
ament_package()

