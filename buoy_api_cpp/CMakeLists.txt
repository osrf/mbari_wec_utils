cmake_minimum_required(VERSION 3.8)
project(buoy_api_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(buoy_interfaces REQUIRED)
find_package(buoy_gazebo REQUIRED)  # for interp1d

add_library(buoy_api INTERFACE)

target_include_directories(buoy_api INTERFACE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)

install(TARGETS buoy_api
  EXPORT "export_buoy_api"
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
  INCLUDES DESTINATION include
)

install(DIRECTORY include/buoy_api/
  DESTINATION include/buoy_api
)

###########################################
# Examples

add_executable(torque_controller examples/torque_controller.cpp)
target_link_libraries(torque_controller PUBLIC buoy_api)
ament_target_dependencies(torque_controller PUBLIC rclcpp buoy_interfaces buoy_gazebo)
target_include_directories(torque_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/examples/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(torque_controller PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS torque_controller
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

###########################################
# for buoy_tests
add_library(torque_policy_interface INTERFACE)
target_compile_definitions(torque_policy_interface INTERFACE POLICY_ONLY)
ament_target_dependencies(torque_policy_interface INTERFACE buoy_gazebo)
target_include_directories(torque_policy_interface INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/examples/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS torque_policy_interface
  EXPORT export_torque_policy_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
  INCLUDES DESTINATION include
)

install(DIRECTORY examples/include/buoy_api/
  DESTINATION include/buoy_api
)

###########################################

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_targets("export_buoy_api")

ament_export_dependencies(rclcpp buoy_interfaces)

# for buoy_tests
ament_export_targets(export_torque_policy_interface)

ament_package()
