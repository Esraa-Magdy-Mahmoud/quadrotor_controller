#include <ros/ros.h>
#include "quadrotor_controller/QuadRotorController.h"
#include "quadrotor_controller/quad_traj.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "quadrotor_controller");
  ros::NodeHandle nodeHandle("~");

  quadrotor_controller::QuadRotorController quadrotorController(nodeHandle);
  quadrotor_controller::quad_traj quadrotor_controller(nodeHandle);

  ros::spin();
  return 0;
}