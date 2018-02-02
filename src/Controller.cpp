#include "quadrotor_controller/Controller.h"


namespace quadrotor_controller
{
    Controller::Controller(double &kp_z,double &kd_z,double &start_time)
    :kpz_(kp_z),kdz_(kd_z),start_time_(start_time)
    {
    }

    Controller::~Controller()
    {
    }

    void Controller::controller(double pose_z,double time_now)
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


