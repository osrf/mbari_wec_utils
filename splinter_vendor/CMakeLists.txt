cmake_minimum_required(VERSION 3.5)

project(splinter_vendor)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)

# TODO(anyone) implement when backported?
# https://github.com/ament/ament_cmake/pull/390
# ament_add_default_options()
# for now, this takes its place:
option(
  BUILD_SHARED_LIBS
    "Global flag to cause add_library() to create shared libraries if on. \
    If set to true, this will cause all libraries to be built shared \
    unless the library was explicitly added as a static library."
  ON
)

macro(build_splinter)
  set(extra_cmake_args)

  if(DEFINED CMAKE_BUILD_TYPE)
    list(APPEND extra_cmake_args -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE})
  endif()

  if(DEFINED CMAKE_TOOLCHAIN_FILE)
    list(APPEND extra_cmake_args "-DCMAKE_TOOLCHAIN_FILE=${CMAKE_TOOLCHAIN_FILE}")
  else()
    list(APPEND extra_cmake_args "-DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}")
  endif()
  list(APPEND extra_cmake_args "-DCMAKE_CXX_FLAGS=${CMAKE_CXX_FLAGS}")

  include(ExternalProject)

  externalproject_add(splinter_3.0
    GIT_REPOSITORY https://github.com/bgrimstad/splinter
    GIT_TAG v3.0
    UPDATE_COMMAND ""
    TIMEOUT 600
    CMAKE_ARGS
      -DCMAKE_INSTALL_PREFIX=${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}_install
      -DEIGEN_DIRECTORY=${EIGEN3_INCLUDE_DIRS}
    PATCH_COMMAND
      ${CMAKE_COMMAND} -E chdir <SOURCE_DIR> git apply -p1 --ignore-space-change --whitespace=nowarn
      ${CMAKE_CURRENT_SOURCE_DIR}/0001-Update-for-newer-Eigen-API-version.patch
      ${CMAKE_CURRENT_SOURCE_DIR}/0002-Bump-CMake-minimum-version-to-suppress-warnings.patch
      ${CMAKE_CURRENT_SOURCE_DIR}/0003-Exclude-installing-Eigen-from-vendored-version.patch
      ${CMAKE_CURRENT_SOURCE_DIR}/0004-Fix-Utilities-Include.patch
  )

  # The external project will install to the build folder, but we'll install that on make install.
  install(
    DIRECTORY
      ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}_install/
    DESTINATION
      ${CMAKE_INSTALL_PREFIX}
    USE_SOURCE_PERMISSIONS
  )

endmacro()

build_splinter()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package(CONFIG_EXTRAS "splinter_vendor-extras.cmake.in")
