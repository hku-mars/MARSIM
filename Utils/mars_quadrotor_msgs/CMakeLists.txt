cmake_minimum_required(VERSION 3.8)
project(quadrotor_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/EsdfMap.msg"
  "msg/FcToOa.msg"
  "msg/OaResult.msg"
  "msg/VioResult.msg"
  "msg/Ctrl.msg"
  "msg/Aec.msg"
  "msg/AuxCommand.msg"
  "msg/Corrections.msg"
  "msg/Gains.msg"
  "msg/OutputData.msg"
  "msg/PositionCommand.msg"
  "msg/PPROutputData.msg"
  "msg/Serial.msg"
  "msg/SO3Command.msg"
  "msg/StatusData.msg"
  "msg/TRPYCommand.msg"
  "msg/Odometry.msg"
  "msg/PolynomialTrajectory.msg"
  "msg/MincoTrajectory.msg"
  "msg/LQRTrajectory.msg"
  "msg/DroneAecInfo.msg"
  "msg/OaManagerDebug.msg"
  "msg/TakeoffLand.msg"
  "msg/Px4ctrlDebug.msg"
  "msg/ServerTime.msg"
  "msg/TrakingPerformance.msg"
  "msg/MpcPositionCommand.msg"
  "msg/QuadrotorState.msg"
  DEPENDENCIES geometry_msgs nav_msgs std_msgs builtin_interfaces
)

add_library(decode_msgs src/decode_msgs.cpp)
add_library(encode_msgs src/encode_msgs.cpp)

target_include_directories(decode_msgs PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(encode_msgs PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(decode_msgs PUBLIC c_std_99 cxx_std_17)
target_compile_features(encode_msgs PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(decode_msgs "Eigen3")
ament_target_dependencies(encode_msgs "Eigen3")

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(decode_msgs "${cpp_typesupport_target}")
target_link_libraries(encode_msgs "${cpp_typesupport_target}")

install(TARGETS decode_msgs encode_msgs
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# Install header files
install(DIRECTORY include/
  DESTINATION include/)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()