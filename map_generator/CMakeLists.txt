cmake_minimum_required(VERSION 3.8)
project(map_generator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)

add_executable(map_pub src/map_publisher.cpp)
target_include_directories(map_pub PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(map_pub PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(
  map_pub
  "rclcpp"
  "std_msgs"
  "sensor_msgs"
  "pcl_conversions"
  "PCL"
)

install(TARGETS map_pub
  DESTINATION lib/${PROJECT_NAME})

# Install resource files
install(DIRECTORY resource/
  DESTINATION share/${PROJECT_NAME}/resource/)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
