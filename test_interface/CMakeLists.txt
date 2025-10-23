cmake_minimum_required(VERSION 3.8)
project(test_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(test_interface_node src/test_interface_node.cpp)
target_include_directories(test_interface_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(test_interface_node PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(
  test_interface_node
  "rclcpp"
  "std_msgs"
  "quadrotor_msgs"
  "geometry_msgs"
  "tf2"
  "tf2_ros"
  "tf2_geometry_msgs"
)

install(TARGETS test_interface_node
  DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch/)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config/)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()