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
         ROS_INFO("Successfully launched node.");
    }

    QuadRotorController::~QuadRotorController()
    {
    }



    bool QuadRotorController::readParameters()
    {

    }
    

}


