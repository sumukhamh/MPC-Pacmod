#include <math.h>
#include <cmath>
#include "publish_control_board.h"

PublishControlBoard::PublishControlBoard()
{
  // Subscribe to messages
  ctrl_cmd_sub = n.subscribe("/control_cmd", 20, &PublishControlBoard::callback_ctrl_cmd, this);  
  speed_rpt_sub = n.subscribe("/pacmod/parsed_tx/vehicle_speed_rpt", 20, &PublishControlBoard::callback_speed_rpt, this);
  // steer_rpt_sub = n.subscribe("/pacmod/parsed_tx/steer_rpt", 20, &PublishControlBoard::callback_steer_rpt, this);
  enable_sub = n.subscribe("/pacmod/as_tx/enable", 20, &PublishControlBoard::callback_pacmod_enable, this);
  set_enable_sub = n.subscribe("/game_control/joy", 20, &PublishControlBoard::callback_set_enable, this);

  // Advertise published messages
  enable_pub = n.advertise<std_msgs::Bool>("/pacmod/as_rx/enable", 20);
  accel_cmd_pub = n.advertise<pacmod_msgs::PacmodCmd>("/pacmod/as_rx/accel_cmd", 20);
  brake_cmd_pub = n.advertise<pacmod_msgs::PacmodCmd>("/pacmod/as_rx/brake_cmd", 20);
  steer_cmd_pub = n.advertise<pacmod_msgs::PositionWithSpeed>("/pacmod/as_rx/steer_cmd", 20); 
}

bool PublishControlBoard::check_is_enabled(const mpc_msgs::Ctrl::ConstPtr& msg)
{
  bool local_enable = false;
  enable_mutex.lock();
  local_enable = pacmod_enable;
  enable_mutex.unlock();
  return local_enable;
}

/*
 * Called when the node receives a message from the enable topic
 */
void PublishControlBoard::callback_pacmod_enable(const std_msgs::Bool::ConstPtr& msg)
{
  enable_mutex.lock();
  pacmod_enable = msg->data;
  enable_mutex.unlock();
}

void PublishControlBoard::publish_accel_message(const mpc_msgs::Ctrl::ConstPtr& msg)
{
  pacmod_msgs::PacmodCmd accel_msg;
  float acc = msg->accel_cmd; 
  //A negative value represents braking and positive value
  if(acc < 0){
    acc = ACCEL_MIN;
  }
  else{
    //A positive value represents acceleration
    //accelerator_cmd_pub takes in positive values
    acc = std::abs(acc); 
    if(acc > ACCEL_MAX){
      acc = ACCEL_MAX;
    }
    if(acc < ACCEL_MIN){
      acc = ACCEL_MIN;
    }
  }
  accel_msg.f64_cmd = acc;
  accel_cmd_pub.publish(accel_msg);
}

void PublishControlBoard::publish_brake_message(const mpc_msgs::Ctrl::ConstPtr& msg)
{
  pacmod_msgs::PacmodCmd brake_msg;
  float brake = msg->accel_cmd;
  //A positive value represents acceleration 
  if(brake > 0){
    brake = BRAKE_MIN;
  }
  else{
    //A negative value represents braking
    //brake_set_position_pub takes in positive values
    brake = std::abs(brake); 
    if(brake > BRAKE_MAX){
      brake = BRAKE_MAX; 
    }
    if(brake < BRAKE_MIN){
      brake = BRAKE_MIN;
    }
  }
  brake_msg.f64_cmd = brake;
  brake_cmd_pub.publish(brake_msg);
}

void PublishControlBoard::publish_steer_message(const mpc_msgs::Ctrl::ConstPtr& msg)
{
  pacmod_msgs::PositionWithSpeed steer_msg;
  float speed_scale = 0.5;
  bool speed_valid = false;
  float current_speed = 0.0;

  speed_mutex.lock();

  if (last_speed_rpt != NULL)
    speed_valid = last_speed_rpt->vehicle_speed_valid;

  if (speed_valid)
    current_speed = last_speed_rpt->vehicle_speed;

  speed_mutex.unlock();

  if (speed_valid)
    speed_scale = STEER_OFFSET - fabs((current_speed / (max_veh_speed * STEER_SCALE_FACTOR))); //Never want to reach 0 speed scale.
  
  float steer_position_rad = msg->steer_cmd * PI / 180;
  
  if (std::abs(steer_position_rad) > MAX_ROT_RAD_DEFAULT){
    if (steer_position_rad < 0){
        steer_position_rad = -MAX_ROT_RAD_DEFAULT;
    }
    else{
        steer_position_rad = MAX_ROT_RAD_DEFAULT;
    }
  }
  steer_msg.angular_position = steer_position_rad;
  steer_msg.angular_velocity_limit = steering_max_speed * speed_scale;
  steer_cmd_pub.publish(steer_msg);
}

/*This function allows us to remove the dependency of pacmod_game_controller and 
 * is meant for enabling only: full joystick controller functionality is not available
 */
void PublishControlBoard::callback_set_enable(const sensor_msgs::Joy::ConstPtr& msg){
  bool local_enable = false;
  enable_mutex.lock();
  local_enable = pacmod_enable;
  enable_mutex.unlock();
  //Enable 
  if(msg->buttons[ENABLE_INDEX] == BUTTON_DOWN){
    std_msgs::Bool bool_pub_msg;
    bool_pub_msg.data = true;
    local_enable = true;
    enable_pub.publish(bool_pub_msg);
  }
  //Disable
  if(msg->buttons[DISABLE_INDEX] == BUTTON_DOWN){
    std_msgs::Bool bool_pub_msg;
    bool_pub_msg.data = false;
    local_enable = false;
    enable_pub.publish(bool_pub_msg);
  }
  enable_mutex.lock();
  pacmod_enable = local_enable;
  enable_mutex.unlock();;
}

/*
 * Called when the node receives a message from the vehicle speed topic
 */
void PublishControlBoard::callback_speed_rpt(const pacmod_msgs::VehicleSpeedRpt::ConstPtr& msg)
{
  speed_mutex.lock();
  last_speed_rpt = msg;
  speed_mutex.unlock();
  //This condition allows us to actuate the brakes once a stop is made. 
  if((std::abs(last_speed_rpt->vehicle_speed) <= BRAKE_DELTA) && (mpc_velocity == 0.0)){
    pacmod_msgs::PacmodCmd stop_brake_msg; 
    stop_brake_msg.f64_cmd = 7.5*BRAKE_MAX/8;
    brake_cmd_pub.publish(stop_brake_msg);
    reset_brake = true;
  }
  else{
    if(reset_brake){
      pacmod_msgs::PacmodCmd stop_brake_msg; 
      stop_brake_msg.f64_cmd = BRAKE_MIN;
      brake_cmd_pub.publish(stop_brake_msg);
      reset_brake = false;
    }
  }
}

// Change message type
void PublishControlBoard::callback_ctrl_cmd(const mpc_msgs::Ctrl::ConstPtr& msg)
{
  try
  {
    /* 
     * Set brake pedal to max if set speed=0 m/s && last_speed_rpt->vehicle_speed is approx 0 m/s
     */
    if(msg->velocity >= 0.0){
      mpc_velocity = msg->velocity; 
    }
    if (check_is_enabled(msg) == true){ // && msg->mode == 1){
      // Accelerator
      publish_accel_message(msg);

      // Brake
      publish_brake_message(msg);
      
      // Steering
      publish_steer_message(msg);
    }
  }
  catch (const std::out_of_range& oor)
  {
    ROS_ERROR("An out-of-range exception was caught. This probably means a wrong autoware mapping.");
  }
}
