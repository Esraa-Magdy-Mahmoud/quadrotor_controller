#ifndef QUADROTORCONTROLLER_H_
#define  QUADROTORCONTROLLER_H_

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <gazebo_msgs/ModelState.h>
#include <visualization_msgs/Marker.h>
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
		void eulerCallback(const geometry_msgs::Vector3StampedConstPtr &eulermsg);
	    void pidController(double pose_z,double time_now);
		void targetMarker();
		void quadMarker();
		//---
 
        //-- nodehandle
		ros::NodeHandle nodeHandle_;

		
        //-- states sub
		ros::Subscriber poseSub_;
		ros::Subscriber eulerSub_;
        //--
		
		ros::Publisher  velPub_;
		geometry_msgs::Twist velmsg_; 
		ros::Publisher visTargetPub_;
		ros::Publisher visQuadPub_;
		
		visualization_msgs::Marker targetvis;
		visualization_msgs::Marker quadvis;



		//-- pid gains--//
		double kpz_ = 10;
		double kdz_ = 0.5;
		double kpx_ = 0.3;
		double kdx_ = 0.5;
		double kpy_ = 0.7;
		double kdy_ = 0.5;
		double kp_roll = 15;
		double kd_roll = 0.4;
		double kp_pitch = 0.4;
		double kd_pitch = 0.4;
		double kp_yaw = 0.7;
		double kd_yaw = 0.7;

		//--
		double thrust_;
		double momentx_;
		double momenty_;
		double momentz_;

	   //--Target point position
		double pos_x_target = 10.0;
		double pos_y_target = 10.0; 
    	double pos_z_target = 10.0;
		double yaw_target = 0.0;
		double dyaw_target= 0.0;

	  //----
		double last_errorx_ = 0.0;
		double error_sumx_  = 0.0;

	    double last_errory_ = 0.0;
		double error_sumy_  = 0.0;

		double last_errorz_ = 0.0;
		double error_sumz_  = 0.0;
        
		double last_erroryaw_ = 0.0;
		double error_sumyaw_  = 0.0;

		double last_erroroll_ = 0.0;
		double error_sumroll_  = 0.0;
		
		double last_errorpitch_ = 0.0;
		double error_sumpitch_  = 0.0;
		
		double start_time_ = 0.0;
		double last_time_  = 0.0;
        double timenow_;


		
		
       

		//--QuadStates---//
		double pose_x_;
		double pose_y_;
		double pose_z_;
        double yaw_;
		double pitch_;
		double roll_;
		double dyaw_;
		double dpitch_;
		double droll_;



	};
	
}




#endif