cmake_minimum_required(VERSION 3.8)
project(waypoint_generator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)

include_directories(include)

add_executable(waypoint_generator src/waypoint_generator.cpp)
target_include_directories(waypoint_generator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(waypoint_generator PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(
  waypoint_generator
  "rclcpp"
  "tf2"
  "tf2_ros"
  "nav_msgs"
)

install(TARGETS waypoint_generator
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()