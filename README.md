# quadrotor_controller

a 3D PID hover controller for the Quadrotor .


## Requirements:
  * Ubuntu 16.04.
  * ROS Kinetic.
  * gazebo 7.7.0.
  * hector_quadrotor.
  
## Setup hector_quadrotor in kinetic:
  ```
  $ cd ~/catkin_ws/src
 ```
  ```
  $ sudo apt-get install ros-kinetic-ros-control
  $ sudo apt-get install ros-kinetic-gazebo-ros-control
  $ sudo apt-get install ros-kinetic-unique-identifier
  $ sudo apt-get install ros-kinetic-geographic-info
  $ sudo apt-get install ros-kinetic-laser-geometry
  $ sudo apt-get install ros-kinetic-tf-conversions
  $ sudo apt-get install ros-kinetic-tf2-geometry-msgs
  $ sudo apt-get install ros-kinetic-joy
  ```
  ```
  $ git clone -b kinetic-devel https://github.com/tu-darmstadt-ros-pkg/hector_quadrotor
  $ git clone -b catkin https://github.com/tu-darmstadt-ros-pkg/hector_localization
  $ git clone -b kinetic-devel https://github.com/tu-darmstadt-ros-pkg/hector_gazebo
  $ git clone -b kinetic-devel https://github.com/tu-darmstadt-ros-pkg/hector_models
  $ git clone -b catkin https://github.com/tu-darmstadt-ros-pkg/hector_slam
  ```
## Experiment the Quadrotor controller node :
 ```
 $ roslaunch quadrotor_controller quadrotor_controller.launch
 $ roslaunch quadrotor_controller one_quad.launch
  ``` 
## Multi-Quadrotors hover :
  ```
 $ roslaunch quadrotor_controller multi_quadrotor_controller.launch
 $ roslaunch quadrotor_controller test.launch
  ```
 
  


