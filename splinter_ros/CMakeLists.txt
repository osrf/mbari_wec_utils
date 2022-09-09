cmake_minimum_required(VERSION 3.8)
project(splinter_ros)

find_package(ament_cmake REQUIRED)
find_package(splinter_vendor REQUIRED)

add_library(splinter1d SHARED src/splinter1d.cpp)
target_link_libraries(splinter1d PRIVATE splinter)
target_include_directories(splinter1d PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)

add_library(splinter2d SHARED src/splinter2d.cpp)
target_link_libraries(splinter2d PRIVATE splinter)
target_include_directories(splinter2d PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)

install(TARGETS splinter1d splinter2d
  EXPORT "export_splinter_ros"
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
  INCLUDES DESTINATION include
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_targets("export_splinter_ros")
ament_package()
