cmake_minimum_required(VERSION 3.8)
project(grid_map_nav2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(grid_map_ros REQUIRED)

set(lib_name ${PROJECT_NAME})

add_library(${lib_name} SHARED src/grid_map_layer.cpp)

target_include_directories(${lib_name} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(${lib_name}
  rclcpp
  rclpy
  nav2_costmap_2d
  pluginlib
  grid_map_ros)

install(TARGETS ${lib_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY include/
  DESTINATION include/)

install(FILES grid_map_layer.xml
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  scripts/grid_map_publisher.py
  DESTINATION lib/${PROJECT_NAME})

pluginlib_export_plugin_description_file(nav2_costmap_2d grid_map_layer.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()