#include <ros/ros.h>
#include <math.h>
#include <thread>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include <MPC.h>
#include "mpc_msgs/Ctrl.h"

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "mpc_controller");
  ros::NodeHandle n;

  MPC mpc(n);

  mpc.run();
}
