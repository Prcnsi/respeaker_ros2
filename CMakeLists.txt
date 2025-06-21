cmake_minimum_required(VERSION 3.5)  # 최소 cmake 버전 지정

# 프로젝트 정의
project(respeaker_ros2)

# 의존성 패키지 찾기
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)  # 필요한 메시지 의존성 추가

# 메시지 파일을 생성하고 빌드하기 위한 설정
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AudioDataWithDOA.msg"  # AudioDataWithDOA.msg 파일을 사용
  DEPENDENCIES std_msgs  # std_msgs를 의존성에 추가
)

# Python 패키지 빌드를 위한 설정 (Python 파일을 사용하고 있으면)
ament_python_install_package()

# 패키지 정의
ament_package()
