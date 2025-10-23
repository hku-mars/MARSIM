cmake_minimum_required(VERSION 3.8)
project(rviz_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz2 REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)

# Qt5 setup
add_definitions(-DQT_NO_KEYWORDS)

# MOC files
qt5_wrap_cpp(MOC_FILES
  src/goal_tool.h
)

# Source files
set(SOURCE_FILES
  src/pose_tool.cpp
  src/goal_tool.cpp
  ${MOC_FILES}
)

include_directories(include)

add_library(${PROJECT_NAME} ${SOURCE_FILES})
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(
  ${PROJECT_NAME}
  "rclcpp"
  "rviz2"
)

target_link_libraries(${PROJECT_NAME} Qt5::Widgets)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(FILES plugin_description.xml
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY media/
  DESTINATION share/${PROJECT_NAME}/media)

install(DIRECTORY icons/
  DESTINATION share/${PROJECT_NAME}/icons)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()