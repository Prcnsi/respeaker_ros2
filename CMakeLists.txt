cmake_minimum_required(VERSION 3.22)
project(respeaker_ros2)

# 필수 패키지 찾기
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)  # Python 지원 추가
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED) 

# 메시지 파일 생성
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AudioDataWithDOA.msg"
  DEPENDENCIES std_msgs builtin_interfaces
)

# Python 노드 설치 (실행 파일로 복사)
install(PROGRAMS src/respeaker_ros2/audio_with_doa_publisher.py
        DESTINATION lib/${PROJECT_NAME}
        RENAME audio_with_doa_node)  # 실행 가능한 이름으로 변경

ament_export_dependencies(rosidl_default_runtime)
ament_package()
