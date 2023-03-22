cmake_minimum_required(VERSION 3.8)
project(radar_detection_cpp)

# Default to C++17.
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set ROS_DISTRO macros
set(ROS_DISTRO $ENV{ROS_DISTRO})
if(${ROS_DISTRO} STREQUAL "rolling")
  add_compile_definitions(ROS_DISTRO_ROLLING)
elseif(${ROS_DISTRO} STREQUAL "galactic")
  add_compile_definitions(ROS_DISTRO_GALACTIC)
elseif(${ROS_DISTRO} STREQUAL "humble")
  add_compile_definitions(ROS_DISTRO_HUMBLE)
endif()

# Export the library using ament_auto_cmake
find_package(ament_cmake_auto REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIRS})
ament_auto_find_build_dependencies(REQUIRED 
  ${${PROJECT_NAME}_BUILD_DEPENDS}
  ${${PROJECT_NAME}_BUILDTOOL_DEPENDS}
) 

# TODO remove if doesn't work
SET(${PROJECT_NAME}_SRC
  src/radar_pcl_to_cam.cpp
  src/radar_detection.cpp
)
# ament_auto_add_library(${PROJECT_NAME} SHARED 
#   ${${PROJECT_NAME}_LIB_SRC}
#   ${${PROJECT_NAME}_LIB_HEADER}
# )

ament_auto_add_executable(radar_to_cam_node
  ${${PROJECT_NAME}_SRC}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
