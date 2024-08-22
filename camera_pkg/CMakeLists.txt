cmake_minimum_required(VERSION 3.5)
project(camera_pkg)

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

if(CMAKE_BUILD_TYPE STREQUAL "Release")
  add_compile_options(-O3 -march=native -DNDEBUG)
elseif(CMAKE_BUILD_TYPE STREQUAL "Debug")
  add_compile_options(-g)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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

install(DIRECTORY launch params urdf rviz DESTINATION share/${PROJECT_NAME})

include_directories(include)

set(image_publisher_dependencies "rclcpp" "sensor_msgs" "OpenCV" "image_transport" "cv_bridge")
add_executable(image_publisher src/image_publisher.cpp)
ament_target_dependencies(image_publisher ${image_publisher_dependencies})
install(TARGETS image_publisher DESTINATION lib/${PROJECT_NAME})

set(static_camera_tf2_broadcaster_dependencies "rclcpp" "geometry_msgs" "tf2" "tf2_ros")
add_executable(static_camera_tf2_broadcaster src/static_camera_tf2_broadcaster.cpp)
ament_target_dependencies(static_camera_tf2_broadcaster ${static_camera_tf2_broadcaster_dependencies})
install(TARGETS static_camera_tf2_broadcaster DESTINATION lib/${PROJECT_NAME})

ament_package()
