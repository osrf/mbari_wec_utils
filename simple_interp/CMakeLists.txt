cmake_minimum_required(VERSION 3.8)
project(simple_interp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)

add_library(interp1d src/interp1d.cpp)
target_compile_features(interp1d PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(interp1d PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)


install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS interp1d
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(
  include
)
ament_export_libraries(
  interp1d
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
