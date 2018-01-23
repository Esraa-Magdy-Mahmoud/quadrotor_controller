#ifndef QUADROTORCONTROLLER_H_
#define  QUADROTORCONTROLLER_H_

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

namespace quadrotor_controller
{
	class QuadRotorController
	{
		public:
		QuadRotorController(ros::NodeHandle& nodeHandle);
		virtual ~QuadRotorController();

		private:
        
		//---Methods definition---//
		bool readParameters();
        //---

        
		ros::NodeHandle nodeHandle_;
        




	};
	
}




#endif