#include <ros/ros.h>
#include "quadrotor_controller/QuadRotorController.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "quadrotor_controller");
  ros::NodeHandle nodeHandle("~");

  quadrotor_controller::QuadRotorController quadrotorController(nodeHandle);

  ros::spin();
  return 0;
}