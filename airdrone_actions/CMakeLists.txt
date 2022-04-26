cmake_minimum_required(VERSION 3.5)
project(airdrone_actions)


# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Offboard.action"
  "action/OffboardRequest.action"
  "action/Setpoint.action"
  "action/TakeVideo.action"
  "srv/OffboardSession.srv"
  "srv/StartOffboard.srv"
  "srv/EndOffboard.srv"
  "srv/TakePicture.srv"
  "srv/StartRecording.srv"
  "srv/StopRecording.srv"
  DEPENDENCIES sensor_msgs
)

ament_package()
