cmake_minimum_required(VERSION 3.5)

project(turtlebot_bringup)

# Default to C++14
set(CMAKE_CXX_STANDARD 14)

#add_compile_options(-Wall)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
  set(LIBKOBUKI_DIR $ENV{LIBKOBUKI_ROOT})
  set(LIBKOBUKI_INC_DIR ${LIBKOBUKI_DIR}/include)
  set(LIBKOBUKI_LIB_DIR ${LIBKOBUKI_DIR}/lib)
else()
  set(LIBKOBUKI_INC_DIR /usr/local/include/kobuki)
  set(LIBKOBUKI_LIB_DIR /usr/local/lib)
endif()

set(LIBKOBUKI_LIBRARIES kobuki)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ./lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ./bin)

include_directories(include src/bringup)
include_directories(${LIBKOBUKI_INC_DIR})
add_definitions(-D_USE_MATH_DEFINES)
link_directories(${LIBKOBUKI_LIB_DIR})


add_library(Turtlebot SHARED src/bringup/Turtlebot.cpp src/bringup/Turtlebot.hpp)
ament_target_dependencies(Turtlebot rclcpp sensor_msgs geometry_msgs nav_msgs tf2_ros)
target_link_libraries(Turtlebot ${LIBKOBUKI_LIBRARIES})

add_executable(turtlebot2 src/bringup/turtlebot2.cpp src/bringup/Turtlebot.cpp src/bringup/Turtlebot.hpp)
ament_target_dependencies(turtlebot2 rclcpp sensor_msgs geometry_msgs nav_msgs tf2_ros)
target_link_libraries(turtlebot2 ${LIBKOBUKI_LIBRARIES})

add_executable(teleop_node src/teleop/teleop_node.cpp)
ament_target_dependencies(teleop_node rclcpp sensor_msgs geometry_msgs nav_msgs tf2_ros)

target_include_directories(turtlebot2 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS turtlebot2
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS teleop_node
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

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

ament_package()
