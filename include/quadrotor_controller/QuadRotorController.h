#ifndef QUADROTORCONTROLLER_H_
#define  QUADROTORCONTROLLER_H_

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <gazebo_msgs/ModelState.h>
#include <string.h>
#include <geometry_msgs/PoseStamped.h>


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
		void poseCallback(const geometry_msgs::PoseStampedConstPtr &posemsg);
	    void pidController(double pose_z,double time_now);
			

        //---
		double thrust_;
			
		double kpz_;
		double kdz_;
        double momentx_;
		double momenty_;
		double momentz_;

	   //Target point position
		double pos_x_target;
		double pos_y_target; 
    	double pos_z_target = 10.0;
	  //
		double last_error_ = 0.0;
		double error_sum_  = 0.0;

		double start_time_;
		double last_time_  = 0.0;
        double timenow_;

        
		ros::NodeHandle nodeHandle_;
		ros::Subscriber poseSub_;
		
       

		//--QuadStates---//
		double pose_x_;
		double pose_y_;
		double pose_z_;




	};
	
}




#endif