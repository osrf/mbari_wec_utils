cmake_minimum_required(VERSION 3.8)
project(buoy_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
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
  ADD_LINTER_TESTS
)

ament_package()
