cmake_minimum_required(VERSION 3.8)

set (PROJECT_NAME zed_gad)
project(${PROJECT_NAME})

# Default to C++17
#if(NOT CMAKE_CXX_STANDARD)
set(CMAKE_CXX_STANDARD 17)
#endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)

#ament_python_install_package(${PROJECT_NAME})

add_executable(${PROJECT_NAME} src/main.cpp src/zed_gad.cpp)


find_library(GAL_C NAMES liboxts-sdk-gal-c.a)
find_library(GAL_CPP NAMES liboxts-sdk-gal-cpp.a)
find_library(OXTS_CORE NAMES liboxts-sdk-core.a)
target_link_libraries(${PROJECT_NAME} ${GAL_C} ${GAL_CPP} ${OXTS_CORE})

ament_target_dependencies(${PROJECT_NAME}
                          rclcpp
                          geometry_msgs
                          nav_msgs
                          sensor_msgs
                          tf2)                        

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  $ENV{EIGEN3_INCLUDE_DIR})

target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

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

ament_package()
