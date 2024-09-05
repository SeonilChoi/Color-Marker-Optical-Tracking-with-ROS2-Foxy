cmake_minimum_required(VERSION 3.5)
project(kalman_filter)

if (NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if (NOT CMAKE_CXX_STANDARD)
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

find_package(Eigen3 REQUIRED)

set(KALMAN_FILTER_SOURCES src/kalman_filter.cpp)
add_library(kalman_filter_library SHARED ${KALMAN_FILTER_SOURCES})
target_link_libraries(kalman_filter_library ${Eigen3_LIBRARIES})

target_include_directories(kalman_filter_library PUBLIC "${PROJECT_SOURCE_DIR}/include")

install(TARGETS kalman_filter_library
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)
    
install(DIRECTORY include/ DESTINATION include)
