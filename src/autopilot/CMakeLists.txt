cmake_minimum_required(VERSION 3.8)
project(autopilot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)


add_executable(autopilot src/AutoPilot.cpp)
ament_target_dependencies(autopilot rclcpp std_msgs geometry_msgs sensor_msgs nav_msgs)

install(TARGETS
	autopilot
	DESTINATION lib/${PROJECT_NAME})


ament_package()
