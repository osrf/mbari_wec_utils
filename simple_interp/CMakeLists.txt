cmake_minimum_required(VERSION 3.8)
project(simple_interp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
ament_find_gtest()

add_library(${PROJECT_NAME} SHARED src/interp1d.cpp src/interp2d.cpp)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

install(
  TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest_executable(test_${PROJECT_NAME}
    tests/test_${PROJECT_NAME}.cpp
  )

  target_link_libraries(test_${PROJECT_NAME} ${PROJECT_NAME})

  ament_add_gtest_test(test_${PROJECT_NAME})
endif()

ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package()
