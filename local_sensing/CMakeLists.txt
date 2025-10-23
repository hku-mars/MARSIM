cmake_minimum_required(VERSION 3.8)
project(local_sensing_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(GLEW REQUIRED)
find_package(OpenGL REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GLFW3 REQUIRED glfw3)

find_package(Boost REQUIRED COMPONENTS filesystem iostreams program_options system serialization)
if(Boost_FOUND)
  include_directories(${Boost_INCLUDE_DIRS})
  link_directories(${Boost_LIBRARY_DIRS})
endif()

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/modules/")
link_directories(${CMAKE_SOURCE_DIR}/lib)

add_library(GLAD include/glad.c)
set(LIBS ${LIBS} GLAD)

include_directories(
  SYSTEM
  include
  ${Eigen_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${OPENGL_INCLUDE_DIR}
  ${GLEW_INCLUDE_DIRS}
  ${glfw3_INCLUDE_DIRS}
)

# Disabled pcl_render_node for now - needs ROS 2 conversion
# add_executable(
#   pcl_render_node
#   src/pointcloud_render_node.cpp
#   include/ikd-Tree/ikd_Tree.cpp
#   include/FOV_Checker/FOV_Checker.cpp
# )

# target_compile_features(pcl_render_node PUBLIC c_std_99 cxx_std_17)
# ament_target_dependencies(pcl_render_node ...)
# target_link_libraries(pcl_render_node ${OpenCV_LIBS})

add_executable(
  opengl_render_node
  src/opengl_render_node.cpp
  include/ikd-Tree/ikd_Tree.cpp
  include/FOV_Checker/FOV_Checker.cpp
)

target_compile_features(opengl_render_node PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(
  opengl_render_node
  "rclcpp"
  "sensor_msgs"
  "geometry_msgs"
  "nav_msgs"
  "quadrotor_msgs"
  "cv_bridge"
  "image_transport"
  "pcl_conversions"
  "ament_index_cpp"
  "Eigen3"
  "OpenCV"
  "PCL"
)

set(glfw3_LIBS /usr/lib/x86_64-linux-gnu/libglfw.so)
target_link_libraries(opengl_render_node
  ${OpenCV_LIBS}
  ${Boost_LIBRARIES}
  ${OPENGL_LIBRARIES}
  ${GLEW_LIBRARIES}
  ${GLFW3_LIBRARIES}
  ${LIBS}
)

install(TARGETS opengl_render_node
  DESTINATION lib/${PROJECT_NAME})

# Install header files
install(DIRECTORY include/
  DESTINATION include/)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()