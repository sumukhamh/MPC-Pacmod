#ifndef PUBLISH_CONTROL_BOARD_REV2_H
#define PUBLISH_CONTROL_BOARD_REV2_H

#include "globals.h"

// Todo: 
// Check callback_pacmod_enable
// Alter launch file
// Add steer rpt 

class PublishControlBoard
{
public:

  PublishControlBoard();
  
  void callback_ctrl_cmd(const mpc_msgs::Ctrl::ConstPtr& msg);
  void callback_speed_rpt(const pacmod_msgs::VehicleSpeedRpt::ConstPtr& msg);
  // void callback_steer_rpt(const pacmod_msgs::SteerRpt::ConstPtr& msg); // Check this
  void callback_pacmod_enable(const std_msgs::Bool::ConstPtr& msg); // Check how to implement this
  void callback_set_enable(const sensor_msgs::Joy::ConstPtr& msg);

  pacmod_msgs::VehicleSpeedRpt::ConstPtr last_speed_rpt = NULL;
  double max_veh_speed = 11.56;
  double steering_max_speed = 6.7;
  bool pacmod_enable;
  float mpc_velocity = 0.0;
  
private:
  bool check_is_enabled(const mpc_msgs::Ctrl::ConstPtr& msg);
  void publish_accel_message(const mpc_msgs::Ctrl::ConstPtr& msg);
  void publish_brake_message(const mpc_msgs::Ctrl::ConstPtr& msg);
  void publish_steer_message(const mpc_msgs::Ctrl::ConstPtr& msg);

  //When vehicle needs to make a stop, the brake is set and this flag is set to true
  bool reset_brake = false;  

  // ROS node handle
  ros::NodeHandle n;

  // ROS publishers
  ros::Publisher accel_cmd_pub;
  ros::Publisher brake_cmd_pub;
  ros::Publisher steer_cmd_pub;
  ros::Publisher enable_pub;

  // ROS subscribers
  ros::Subscriber ctrl_cmd_sub;
  ros::Subscriber speed_rpt_sub;
  // ros::Subscriber steer_rpt_sub;
  ros::Subscriber enable_sub;
  ros::Subscriber set_enable_sub;
};

#endif
