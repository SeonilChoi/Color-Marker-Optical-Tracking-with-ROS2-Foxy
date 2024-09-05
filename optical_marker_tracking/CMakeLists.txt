cmake_minimum_required(VERSION 3.5)
project(optical_marker_tracking)

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
find_package(optical_tracking_msgs REQUIRED)
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

install(DIRECTORY launch params urdf meshes DESTINATION share/${PROJECT_NAME})

include_directories(include)

link_directories(${kalman_filter_library})
include_directories(${kalman_filter_library_INCLUDE_DIRS})

set(marker_detector_dependencies "rclcpp" "sensor_msgs" "optical_tracking_msgs" "OpenCV" "image_transport" "cv_bridge")
add_executable(marker_detector src/marker_detector.cpp)
ament_target_dependencies(marker_detector ${marker_detector_dependencies})
install(TARGETS marker_detector DESTINATION lib/${PROJECT_NAME})

set(marker_matcher_dependencies "rclcpp" "sensor_msgs" "optical_tracking_msgs" "OpenCV" "image_transport" "cv_bridge")
add_executable(marker_matcher src/marker_matcher.cpp)
ament_target_dependencies(marker_matcher ${marker_matcher_dependencies})
install(TARGETS marker_matcher DESTINATION lib/${PROJECT_NAME})

set(marker_triangulator_dependencies "rclcpp" "optical_tracking_msgs")
add_executable(marker_triangulator src/marker_triangulator.cpp)
ament_target_dependencies(marker_triangulator ${marker_triangulator_dependencies})
install(TARGETS marker_triangulator DESTINATION lib/${PROJECT_NAME})

find_library(kalman_filter_library NAMES kalman_filter_library PATHS /usr/local/lib)
find_path(kalman_filter_library_INCLUDE_DIRS NAMES kalman_filter/kalman_filter.hpp PATHS usr/local/include)

set(marker_tracker_dependencies "rclcpp" "optical_tracking_msgs" "tf2")
add_executable(marker_tracker src/marker_tracker.cpp)
ament_target_dependencies(marker_tracker ${marker_tracker_dependencies})
target_link_libraries(marker_tracker ${kalman_filter_library})
install(TARGETS marker_tracker DESTINATION lib/${PROJECT_NAME})

set_target_properties(marker_tracker PROPERTIES
    INSTALL_RPATH_USE_LINK_PATH TRUE
    INSTALL_RPATH "${kalman_filter_library}")

set(marker_tf2_broadcaster_dependencies "rclcpp" "optical_tracking_msgs" "geometry_msgs" "tf2" "tf2_ros")
add_executable(marker_tf2_broadcaster src/marker_tf2_broadcaster.cpp)
ament_target_dependencies(marker_tf2_broadcaster ${marker_tf2_broadcaster_dependencies})
install(TARGETS marker_tf2_broadcaster DESTINATION lib/${PROJECT_NAME})

set(object_tf2_broadcaster_dependencies "rclcpp" "geometry_msgs" "tf2" "tf2_ros")
add_executable(object_tf2_broadcaster src/object_tf2_broadcaster.cpp)
ament_target_dependencies(object_tf2_broadcaster ${object_tf2_broadcaster_dependencies})
install(TARGETS object_tf2_broadcaster DESTINATION lib/${PROJECT_NAME})

ament_package()


