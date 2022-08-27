/*
* nada
*/

#include "stingray_hw_interface.h"
#include "robot_fin_state.h"
#include <ros/ros.h>
#include <ros_control_boilerplate/generic_hw_control_loop.h>
#include <cmath>
//#include <controller_manager/controller_manager.h>

//class StingrayHWInterface;
//controller_manager - couldn't find expected controller_manager ros interface
//insufficient resources to load all controllers
//not loading proper controllers config file and controller manager is unable to find them
//launching controller names appending the namespace and not loading ros control interface with that same namespace
int main(int argc,char** argv){
    ros::init(argc,argv,"hardware_interface");
    ros::NodeHandle nh;
    ros::CallbackQueue queue;
    nh.setCallbackQueue(&queue);

    std::shared_ptr<StingrayHWInterface> interface (new StingrayHWInterface(nh));
    ros_control_boilerplate::GenericHWControlLoop control_loop(nh,interface);
    ros::AsyncSpinner spinner(4,&queue);
    spinner.start();
    //start stingray_controller action server before beginning wave model
    ROS_INFO_STREAM("Running control_loop");
    control_loop.run();
    wave_model::RobotFinState waveModel = wave_model::RobotFinState(nh);
    //start on a new thread
    ROS_INFO_STREAM("Running wave model");
    waveModel.run();
    spinner.stop();
    return 0;
}
