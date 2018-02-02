#ifndef CONTROLLER_H
#define CONTROLLER_H 

namespace quadrotor_controller
{
	 class Controller
	{
		public:
			Controller(double &kp_z,double &kd_z,double &start_time);
			virtual ~Controller();



			void controller(double pose_z,double time_now);
			double thrust_;
		private:

			
			double momentx_;
			double momenty_;
			double momentz_;

			//Target point
			double pose_x_;
			double pose_y_; 
    		double pose_z_ = 10.0;
			//
			double last_error_ = 0.0;
			double error_sum_  = 0.0;

			double start_time_;
			double last_time_  = 0.0;
			

			//--
			double kpz_;
			double kdz_;




	};
}





#endif