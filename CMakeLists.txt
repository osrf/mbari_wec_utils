cmake_minimum_required(VERSION 3.8)
project(buoy_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BCRecord.msg"
  "msg/PBCommandResponse.msg"
  "msg/PBRecord.msg"
  "msg/PCRecord.msg"
  "msg/SCRecord.msg"
  "msg/TFRecord.msg"
  "msg/XBRecord.msg"
  "msg/PBWatch.msg"
  "srv/BCResetCommand.srv"
  "srv/BenderCommand.srv"
  "srv/BusCommand.srv"
  "srv/GainCommand.srv"
  "srv/PCBattSwitchCommand.srv"
  "srv/PCBiasCurrCommand.srv"
  "srv/PCChargeCurrLimCommand.srv"
  "srv/PCDrawCurrLimCommand.srv"
  "srv/PCPackRateCommand.srv"
  "srv/PCRetractCommand.srv"
  "srv/PCScaleCommand.srv"
  "srv/PCStdDevTargCommand.srv"
  "srv/PCVTargMaxCommand.srv"
  "srv/PCWindCurrCommand.srv"
  "srv/PumpCommand.srv"
  "srv/SCPackRateCommand.srv"
  "srv/SCResetCommand.srv"
  "srv/TetherCommand.srv"
  "srv/TFResetCommand.srv"
  "srv/TFSetActualPosCommand.srv"
  "srv/TFSetChargeModeCommand.srv"
  "srv/TFSetCurrLimCommand.srv"
  "srv/TFSetModeCommand.srv"
  "srv/TFSetPosCommand.srv"
  "srv/TFSetStateMachineCommand.srv"
  "srv/TFWatchDogCommand.srv"
  "srv/ValveCommand.srv"
  DEPENDENCIES std_msgs
)


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
