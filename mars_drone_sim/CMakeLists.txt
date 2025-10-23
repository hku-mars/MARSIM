cmake_minimum_required(VERSION 3.8)
project(mars_drone_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include)

add_executable(quadrotor_dynamics_node src/quadrotor_dynamics_node.cpp)
target_include_directories(quadrotor_dynamics_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(quadrotor_dynamics_node PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(
  quadrotor_dynamics_node
  "rclcpp"
  "nav_msgs"
  "std_msgs"
  "geometry_msgs"
  "sensor_msgs"
  "Eigen3"
)

install(TARGETS quadrotor_dynamics_node
  DESTINATION lib/${PROJECT_NAME})

# Install header files
install(DIRECTORY include/
  DESTINATION include/)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch/)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()