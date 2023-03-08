cmake_minimum_required(VERSION 3.8)
project(radar_detection_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(image_transport REQUIRED)
find_package(Boost REQUIRED)
find_package(novatel_oem7_msgs REQUIRED)
find_package(delphi_esr_msgs REQUIRED)
find_package(autoware_auto_perception_msgs REQUIRED)

add_executable(radar_to_cam_node src/radar_pcl_2_cam.cpp)
ament_target_dependencies(radar_to_cam_node rclcpp std_msgs vision_msgs
visualization_msgs Eigen3 tf2_ros image_transport Boost delphi_esr_msgs novatel_oem7_msgs autoware_auto_perception_msgs)


install(TARGETS
  radar_to_cam_node
  DESTINATION lib/${PROJECT_NAME}
  )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories(${rclcpp_INCLUDE_DIRS}
                    ${vision_msgs_INCLUDE_DIRS})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
