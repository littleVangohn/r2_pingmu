cmake_minimum_required(VERSION 3.8)
project(weapon_docking)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(OpenCV REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME} "action/WeaponDock.action")

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

add_library(weapon_docking_control src/docking_control.cpp)
target_include_directories(weapon_docking_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(weapon_docking_control geometry_msgs)

add_library(weapon_docking_processor src/weapon_docking_processor.cpp)
target_link_libraries(weapon_docking_processor
  ${OpenCV_LIBRARIES}
)

add_executable(weapon_docking_node src/main.cpp src/weapon_docking_node.cpp)
ament_target_dependencies(weapon_docking_node rclcpp rclcpp_action geometry_msgs std_msgs)
target_link_libraries(weapon_docking_node weapon_docking_processor weapon_docking_control)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(weapon_docking_node ${cpp_typesupport_target})

install(TARGETS weapon_docking_node DESTINATION lib/${PROJECT_NAME})
install(TARGETS weapon_docking_processor weapon_docking_control DESTINATION lib)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_docking_control test/test_docking_control.cpp)
  if(TARGET test_docking_control)
    target_link_libraries(test_docking_control weapon_docking_control)
    ament_target_dependencies(test_docking_control geometry_msgs)
  endif()
endif()

ament_package()
