cmake_minimum_required(VERSION 3.8)
project(grid_map_nav2_mppi)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(lib_name ${PROJECT_NAME})

add_library(${lib_name} SHARED src/grid_map_critic.cpp)

target_include_directories(${lib_name} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(grid_map_ros REQUIRED)

ament_target_dependencies(${lib_name}
  rclcpp
  nav2_costmap_2d
  grid_map_ros
  grid_map_msgs)


set(lib_name ${PROJECT_NAME})


install(TARGETS ${lib_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY include/
  DESTINATION include/)

install(FILES grid_map_critic.xml
  DESTINATION share/${PROJECT_NAME})


pluginlib_export_plugin_description_file(nav2_costmap_2d grid_map_critic.xml)


# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_add_gtest(test_grid_map_critic test/test_grid_map_critic.cpp)
  target_link_libraries(test_grid_map_critic
    ${lib_name}
    /opt/ros/jazzy/lib/libmppi_controller.so)
  ament_target_dependencies(test_grid_map_critic
    rclcpp
    nav2_costmap_2d
    grid_map_ros
    grid_map_msgs)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
