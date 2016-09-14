cmake_minimum_required(VERSION 2.8.3)
project(jenny_control)

find_package(catkin 
  REQUIRED COMPONENTS 
  actionlib_msgs
  geometry_msgs
  industrial_msgs
  nav_msgs
  sensor_msgs
  std_msgs 
  genmsg 
  message_generation 
  roscpp 
  rospy 
)

find_package(Boost REQUIRED COMPONENTS system)

set(rnr-jenny_DIR /usr/local/lib/cmake/rnr)
find_package(rnr-jenny
  REQUIRED
)

catkin_python_setup()

add_message_files(
  DIRECTORY msg
  FILES 
  #    Alarms.msg
  #  AmbientLightSensorCaps.msg
  #  AuxPwr.msg
  #  Caps.msg
  #  Dimensions.msg
  #  DutyCycle.msg
  #  Dynamics.msg
  #  Gpio.msg
  #  IlluminanceState.msg
  #  ImuAlt.msg
  #  ImuCaps.msg
  #  MotorCtlrHealth.msg
  #  MotorHealth.msg
  #  Path2D.msg
  #  Pose2DStamped.msg
  #  PowertrainCaps.msg
  #  ProductInfo.msg
  #  RangeState.msg
  #  RobotStatusExtended.msg
  #  RobotTrajectory2D.msg
  #  RobotTrajectoryPoint2D.msg
  #  ToFSensorCaps.msg
  Velocity.msg
)

add_service_files(
  DIRECTORY srv
  FILES
  #  ConfigGpio.srv
  #  EStop.srv
  #  Freeze.srv
  #  GetCaps.srv
  #  GetIlluminance.srv
  #  GetImu.srv
  #  GetImuAlt.srv
  #  GetProductInfo.srv
  #  GetRange.srv
  #  IsAlarmed.srv
  #  IsDescLoaded.srv
  #  ReadGpio.srv
  #  Release.srv
  #  ReloadConfig.srv
  #  ResetEStop.srv
  #  SetAuxPwr.srv
  #  SetDutyCycles.srv
  SetRobotMode.srv
  SetVelocities.srv
  Stop.srv
  #  WriteGpio.srv
)

#add_action_files(
#DIRECTORY action 
#FILES 
#Calibrate.action
#)

## Generate added messages/services/actions with any dependencies listed here
generate_messages(
  DEPENDENCIES
    actionlib_msgs
    geometry_msgs
    industrial_msgs
    nav_msgs
    sensor_msgs
    std_msgs
)

catkin_package(
  CATKIN_DEPENDS actionlib_msgs
)

include_directories(
  include 
  ../include
  ${rnr-jenny_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(jenny_controller
  src/jenny_controller/jenny_controller.cpp
  src/jenny_controller/jenny_controller_main.cpp
)

#add_executable(jenny_gps
  #src/jenny_control/jenny_gps.cpp
  #src/jenny_control/jenny_gps_main.cpp
#)

#add_executable(jenny_imu
  #src/jenny_control/jenny_imu.cpp
  #src/jenny_control/jenny_imu_main.cpp
#)

#add_executable(jenny_teleop
  #  src/jenny_teleop/jenny_teleop.cpp
  #src/jenny_teleop/jenny_teleop_main.cpp
#)

target_link_libraries(jenny_controller
  jenny
  ${rnr-jenny_LIBRARIES}
  ${catkin_LIBRARIES}
)

#target_link_libraries(jenny_gps 
#  jenny
#  ${rnr-jenny_LIBRARIES}
#  ${catkin_LIBRARIES}
#)

#target_link_libraries(jenny_imu 
#  jenny
#  ${rnr-jenny_LIBRARIES}
#  ${catkin_LIBRARIES}
#)

#target_link_libraries(jenny_teleop 
#  jenny
#  ${rnr-jenny_LIBRARIES}
#  ${catkin_LIBRARIES}
#)

add_dependencies(jenny_controller jenny_control_gencpp)
#add_dependencies(jenny_gps jenny_control_gencpp)
#add_dependencies(jenny_imu jenny_control_gencpp)
#add_dependencies(jenny_teleop jenny_control_gencpp)

install(
  PROGRAMS 
    scripts/jenny_panel
    script/jenny_gps.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
