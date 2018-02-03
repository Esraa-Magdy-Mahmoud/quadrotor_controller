#include "quadrotor_controller/QuadRotorController.h"

namespace quadrotor_controller
{
    QuadRotorController::QuadRotorController(ros::NodeHandle& nodeHandle ):nodeHandle_(nodeHandle)
    {
        if(!readParameters())
        {
            ROS_ERROR("Could not read parameters.");
            ros::requestShutdown();
        }

        //--subscribers--//
        poseSub_ = nodeHandle_.subscribe("/pose", 10,&QuadRotorController::poseCallback, this);
        ROS_INFO("Successfully launched node.");
        timenow_ = ros::Time::now().toSec();
        
       

    }

    QuadRotorController::~QuadRotorController()
    {
    }



    bool QuadRotorController::readParameters()
    {
        return true;

    }

    void QuadRotorController::poseCallback(const geometry_msgs::PoseStampedConstPtr &posemsg)
    {
         pose_x_ = posemsg->pose.position.x;
         pose_y_ = posemsg->pose.position.y;
         pose_z_ = posemsg->pose.position.z;

    }
    void QuadRotorController::pidController(double pose_z,double time_now)
    {
        double dt = time_now - last_time_;
        if (dt == 0) 
        {
            thrust_ = 0.0;
        }
        last_time_ = time_now ;
        double error_z   = pose_z_ - pose_z;
        error_sum_ += (error_z*dt) ;
        double derror_z  = error_z - last_error_;
        last_error_ = error_z;

        thrust_ = (kpz_*error_z)+(kdz_*derror_z);
       

    }
    

}


