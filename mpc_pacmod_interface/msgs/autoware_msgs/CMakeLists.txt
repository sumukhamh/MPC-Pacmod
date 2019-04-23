cmake_minimum_required(VERSION 2.8.3)
project(autoware_msgs)

find_package(catkin REQUIRED COMPONENTS
	message_generation
	std_msgs
	geometry_msgs
	sensor_msgs
)


## Generate messages in the 'msg' folder
add_message_files(
  FILES
  CanInfo.msg
  ControlCommand.msg
  VehicleCmd.msg

## Runtime Manager ##
  accel_cmd.msg
  brake_cmd.msg
  indicator_cmd.msg
  lamp_cmd.msg
  steer_cmd.msg

## Remote Control
  RemoteCmd.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  std_msgs
  geometry_msgs
  sensor_msgs
)
