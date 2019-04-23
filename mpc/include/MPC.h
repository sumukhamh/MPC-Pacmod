#ifndef MPC_H
#define MPC_H

#include <vector>
#include <ros/ros.h>
#include <mpc_msgs/Trajectory.h>
#include <mpc_msgs/Ctrl.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <pacmod_msgs/VehicleSpeedRpt.h>
#include "tf/transform_datatypes.h"
#include <tf/LinearMath/Matrix3x3.h>
#include "geometry_msgs/Quaternion.h"
#include <MPC_Util.h>

class MPC {
private:
  Eigen::VectorXd current_state = Eigen::VectorXd(6);
  Eigen::VectorXd coefficients;
  float reference_velocity = 8.94; // 20mph -> 8.94m/s
  
  float current_velocity;
  vector<double> trajectory_x, trajectory_y;
  Pose current_pose;

  void update_trajectory();
  double steer_value;
  double accel_value;
  
  ros::Subscriber trajectory_sub;
  ros::Subscriber velocity_sub;
  ros::Subscriber pose_sub;
  ros::Publisher control_cmd_pub;

public:
  MPC();
  MPC(ros::NodeHandle n);
  virtual ~MPC();
  
  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  
  void callback_trajectory(const geometry_msgs::PoseArray::ConstPtr& trajectory_msg);
  void callback_pose(const geometry_msgs::Pose::ConstPtr& pose_msg);
  void callback_velocity(const pacmod_msgs::VehicleSpeedRpt::ConstPtr& pose_msg);
  void publish_commands(double& velocity, double& steer, double& accel);

  void Solve();
  void run();
};

#endif /* MPC_H */
