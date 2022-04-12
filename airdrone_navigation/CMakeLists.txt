cmake_minimum_required(VERSION 3.5)
project(airdrone_navigation)

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
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(airdrone_actions REQUIRED)
find_package(OpenCV REQUIRED)
find_package(vision_msgs REQUIRED)

# Add Nodes
#add_executable(LandingPositionFilter src/Landing/landing_position_filter.cpp)
#ament_target_dependencies( LandingPositionFilter rclcpp std_msgs geometry_msgs px4_msgs)

# add_executable(LandingTargetDetector src/Landing/landing_target_detector_copy.cpp)
# ament_target_dependencies( LandingTargetDetector rclcpp rclcpp_action rclcpp_components std_msgs geometry_msgs px4_msgs cv_bridge image_transport airdrone_actions)

add_executable(SurfDetector src/Landing/target_detector.cpp)
ament_target_dependencies(SurfDetector rclcpp rclcpp_action rclcpp_components std_msgs geometry_msgs px4_msgs cv_bridge image_transport airdrone_actions OpenCV vision_msgs)


## Offboard Action Server and Client ##
add_library(offboard_server SHARED
  src/Navigation/offboard_control_server.cpp)

target_compile_definitions(offboard_server
  PRIVATE "ACTION_TUTORIALS_CPP_BUILDING_DLL")
ament_target_dependencies(offboard_server
  "airdrone_actions"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "px4_msgs"
  "geometry_msgs"
  )
rclcpp_components_register_node(offboard_server PLUGIN "offboard_control_server::OffboardServer" EXECUTABLE offboard_control_server)

install(TARGETS
offboard_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

add_library(offboard_client SHARED
  src/Navigation/offboard_control_client.cpp)
target_compile_definitions(offboard_client
  PRIVATE "ACTION_TUTORIALS_CPP_BUILDING_DLL")
ament_target_dependencies(offboard_client
  "airdrone_actions"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "px4_msgs")
rclcpp_components_register_node(offboard_client PLUGIN "offboard_control_server::OffboardClient" EXECUTABLE offboard_control_client)
###

install(TARGETS
  offboard_client
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)

# Install Targets
install(TARGETS 
	#LandingPositionFilter
  #LandingTargetDetector
  #LandingFSM
  SurfDetector
  #SetPointChecker
  #UserInterface
  #SetpointPublisher

	DESTINATION lib/${PROJECT_NAME})


install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY
  worlds
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY
  models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
