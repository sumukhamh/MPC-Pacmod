#include "publish_control_board.h"
#include "globals.h"

/*
 * Main method running the ROS Node
 */
int main(int argc, char *argv[])
{
  ros::init(argc, argv, "pacmod_interface");
  ros::AsyncSpinner spinner(2);
  ros::NodeHandle priv("~");

  // Wait for time to be valid
  while (ros::Time::now().nsec == 0);
  
  PublishControlBoard publish_control_board;

  spinner.start();
  ros::waitForShutdown();

  return 0;
}

