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
        eulerSub_= nodeHandle_.subscribe("/euler",10,&QuadRotorController::eulerCallback,this);

        velPub_  = nodeHandle_.advertise<geometry_msgs::Twist>("/cmd_vel",10);
        visTargetPub_ = nodeHandle_.advertise<visualization_msgs::Marker>("/target_marker",10);
        visQuadPub_ =   nodeHandle_.advertise<visualization_msgs::Marker>("/quad_marker",10);
        ROS_INFO("Successfully launched node.");
        
        
       
        
       

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
    
    void QuadRotorController::eulerCallback(const geometry_msgs::Vector3StampedConstPtr &eulermsg)
    {
        yaw_   = eulermsg->vector.z;
        pitch_ = eulermsg->vector.y;
        roll_  = eulermsg->vector.x;
        timenow_ = ros::Time::now().toSec();
        QuadRotorController::pidController(pose_z_,timenow_);
        QuadRotorController::targetMarker();
        QuadRotorController::quadMarker();

        velPub_.publish(velmsg_);
        visTargetPub_.publish(targetvis);
        visQuadPub_.publish(quadvis);


    }


    void QuadRotorController::pidController(double pose_z,double time_now)
    {
        double dt = time_now - last_time_;
        if (dt == 0) 
        {
            thrust_ = 0.0;
        }
        last_time_ = time_now ;
        double error_z   = pos_z_target - pose_z_;
        error_sumz_ += (error_z*dt) ;
        double derror_z  = error_z - last_errorz_;
        last_errorz_ = error_z;
        double ddz = (kpz_*error_z)+(kdz_*derror_z);

        double error_y   = pos_y_target - pose_y_;
        error_sumy_ += (error_y*dt) ;
        double derror_y  = error_y - last_errory_;
        last_errory_ = error_y;
        double ddy = (kpy_*error_y)+(kdy_*derror_y);

        double error_x   = pos_x_target - pose_x_;
        error_sumx_ += (error_x*dt) ;
        double derror_x  = error_x - last_errorx_;
        last_errorx_ = error_x;
        double ddx = (kpx_*error_x)+(kdx_*derror_x);

        double roll_des   =(1/9.8)*((ddx*yaw_target)-ddy); 
        double pitch_des  =(1/9.8)*(ddx+(ddy*yaw_target));

        thrust_ = ddz;

        double error_roll   = roll_des - roll_;
        error_sumroll_ += (error_roll*dt) ;
        double derror_roll  = error_roll - last_erroroll_;
        last_erroroll_ = error_roll;
        double ddroll = (kp_roll*error_roll)+(kd_roll*derror_roll);

        double error_pitch   = pitch_des - pitch_;
        error_sumpitch_ += (error_pitch*dt) ;
        double derror_pitch  = error_pitch - last_errorpitch_;
        last_errorpitch_ = error_pitch;
        double ddpitch = (kp_pitch*error_pitch)+(kd_pitch*derror_pitch);

        double error_yaw   = yaw_target - yaw_;
        error_sumyaw_ += (error_yaw*dt) ;
        double derror_yaw = error_yaw - last_erroryaw_;
        last_erroryaw_ = error_yaw;
        double ddyaw = (kp_yaw*error_yaw)+(kd_yaw*derror_yaw);

        momentx_ = ddroll;
        momenty_ = ddpitch;
        momentz_ = ddyaw;

        velmsg_.linear.x = ddx;
        velmsg_.linear.y = ddy;
        velmsg_.linear.z = thrust_;
        velmsg_.angular.x = 0.0;
        velmsg_.angular.y = 0.0;
        velmsg_.angular.z = 0.0;

        
       

    }
    void QuadRotorController::targetMarker()
    {
        targetvis.header.frame_id = "world";
        targetvis.header.stamp = ros::Time();
        targetvis.ns = "target";
        targetvis.id = 0;
        targetvis.type = visualization_msgs::Marker::SPHERE;
        targetvis.action = visualization_msgs::Marker::ADD;
        targetvis.pose.position.x = pos_x_target;
        targetvis.pose.position.y = pos_y_target;
        targetvis.pose.position.z = pos_z_target;  
        targetvis.scale.x = 1;
        targetvis.scale.y = 1;
        targetvis.scale.z = 1;
        targetvis.color.a = 1.0; // Don't forget to set the alpha!
        targetvis.color.r = 0.0;
        targetvis.color.g = 1.0;
        targetvis.color.b = 1.0;
    }
     void QuadRotorController::quadMarker()
    {
        quadvis.header.frame_id = "world";
        quadvis.header.stamp = ros::Time();
        quadvis.ns = "quad";
        quadvis.id = 1;
        quadvis.type = visualization_msgs::Marker::SPHERE;
        quadvis.action = visualization_msgs::Marker::ADD;
        quadvis.pose.position.x = pose_x_;
        quadvis.pose.position.y = pose_x_;
        quadvis.pose.position.z = pose_x_;  
        quadvis.scale.x = 1;
        quadvis.scale.y = 1;
        quadvis.scale.z = 1;
        quadvis.color.a = 1.0; // Don't forget to set the alpha!
        quadvis.color.r = 1.0;
        quadvis.color.g = 0.0;
        quadvis.color.b = 0.0;
    }
    

}


