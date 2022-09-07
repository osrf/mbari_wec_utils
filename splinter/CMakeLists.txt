cmake_minimum_required(VERSION 3.8)
project(splinter)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)

include(ExternalProject)
set(EXTERNAL_INSTALL_LOCATION ${CMAKE_SOURCE_DIR}/splinter)
# lint_cmake: -readability/wonkycase
ExternalProject_Add(splinter
    GIT_REPOSITORY https://github.com/bgrimstad/splinter
    GIT_TAG 2c877c0d7f68025764f7af048cc609e5c05080e0  # v3.0
    CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${EXTERNAL_INSTALL_LOCATION}
      -DEIGEN_DIRECTORY=${EIGEN3_INCLUDE_DIRS}
    PATCH_COMMAND
      echo 'Fixing Usage of Eigen SparseVector::InnerIterator' &&
      echo '  for the following file:' &&
      echo '${CMAKE_BINARY_DIR}/splinter-prefix/src/splinter/src/bsplinebasis.cpp' &&
      sed -i "s|for \(SparseMatrix::InnerIterator it\(Ji,k\)|for \(SparseVector::InnerIterator it\(Ji,k\)|g"
        ${CMAKE_BINARY_DIR}/splinter-prefix/src/splinter/src/bsplinebasis.cpp
)

add_custom_target(touch_ignore ALL
                  COMMAND ${CMAKE_COMMAND} -E touch
                  ${EXTERNAL_INSTALL_LOCATION}/include/SPLINTER/AMENT_IGNORE)
add_dependencies(touch_ignore splinter)

add_custom_target(copy_readme ALL
                  COMMAND ${CMAKE_COMMAND} -E copy
                  ${CMAKE_BINARY_DIR}/splinter-prefix/src/splinter/README.md
                  ${CMAKE_SOURCE_DIR}/README_SPLINTER.md)
add_dependencies(copy_readme splinter)

add_custom_target(copy_credits ALL
                  COMMAND ${CMAKE_COMMAND} -E copy
                  ${CMAKE_BINARY_DIR}/splinter-prefix/src/splinter/CREDITS.md
                  ${CMAKE_SOURCE_DIR})
add_dependencies(copy_credits splinter)

add_custom_target(mkdir_assets ALL
                  COMMAND ${CMAKE_COMMAND} -E make_directory
                  ${CMAKE_SOURCE_DIR}/assets)
add_dependencies(mkdir_assets splinter)
add_custom_target(copy_assets ALL
                  COMMAND ${CMAKE_COMMAND} -E copy
                  ${CMAKE_BINARY_DIR}/splinter-prefix/src/splinter/assets/*
                  ${CMAKE_SOURCE_DIR}/assets)
add_dependencies(copy_assets mkdir_assets)

add_custom_target(mkdir_docs ALL
                  COMMAND ${CMAKE_COMMAND} -E make_directory
                  ${CMAKE_SOURCE_DIR}/docs)
add_dependencies(mkdir_docs splinter)
add_custom_target(copy_docs ALL
                  COMMAND ${CMAKE_COMMAND} -E copy
                  ${CMAKE_BINARY_DIR}/splinter-prefix/src/splinter/docs/*
                  ${CMAKE_SOURCE_DIR}/docs)
add_dependencies(copy_docs mkdir_docs)

add_custom_target(rm_bundled_eigen ALL
                  COMMAND ${CMAKE_COMMAND} -E remove_directory
                  ${EXTERNAL_INSTALL_LOCATION}/include/SPLINTER/Eigen
                  ${EXTERNAL_INSTALL_LOCATION}/include/SPLINTER/unsupported)
add_dependencies(rm_bundled_eigen copy_docs)
add_custom_target(link_target_includes ALL
                  COMMAND ${CMAKE_COMMAND} -E create_symlink
                  ${EXTERNAL_INSTALL_LOCATION}/include/SPLINTER
                  ${CMAKE_SOURCE_DIR}/include/splinter)
add_dependencies(link_target_includes rm_bundled_eigen)

add_custom_target(link_target_libs ALL
                  COMMAND ${CMAKE_COMMAND} -E create_symlink
                  ${EXTERNAL_INSTALL_LOCATION}/lib
                  ${CMAKE_SOURCE_DIR}/lib)
add_dependencies(link_target_libs link_target_includes)

include_directories(${EIGEN3_INCLUDE_DIRS})

add_library(splinter1d SHARED src/splinter1d.cpp)
target_link_libraries(splinter1d PUBLIC ${CMAKE_SOURCE_DIR}/lib/libsplinter-3-0.so)
target_include_directories(splinter1d PUBLIC
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
add_dependencies(splinter1d link_target_libs)

add_library(splinter2d SHARED src/splinter2d.cpp)
target_link_libraries(splinter2d PUBLIC ${CMAKE_SOURCE_DIR}/lib/libsplinter-3-0.so)
target_include_directories(splinter2d PUBLIC
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
add_dependencies(splinter2d link_target_libs)

install(TARGETS splinter1d
  EXPORT export_splinter1d
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
  INCLUDES DESTINATION include
)

install(TARGETS splinter2d
  EXPORT export_splinter2d
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
  INCLUDES DESTINATION include
)

install(FILES lib/libsplinter-3-0.so
  DESTINATION lib)

install(
  DIRECTORY include/splinter_ros/
  DESTINATION include/splinter_ros
)

ament_export_targets(export_splinter1d export_splinter2d)
ament_export_libraries(lib/libsplinter-3-0.so)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
