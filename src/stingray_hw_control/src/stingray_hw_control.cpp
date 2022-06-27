/*
* nada
*/

#include "stingray_hw_interface.h"
#include <ros/ros.h>
#include <ros_control_boilerplate/generic_hw_control_loop.h>
#include <cmath>
//#include <controller_manager/controller_manager.h>

class StingrayHWInterface;

int main(int argc,char** argv){
    //need parent name for accessing param server
    ros::init(argc,argv,"hardware_interface");
    ros::NodeHandle nh;
    //std::shared_ptr<urdf::Model> urdfModel (new urdf::Model());
    std::shared_ptr<StingrayHWInterface> interface (new StingrayHWInterface(nh));
    ros_control_boilerplate::GenericHWControlLoop control_loop(nh,interface);
    ros::AsyncSpinner spinner(1);
    spinner.start();
    //ros::waitForShutdown();
    control_loop.run();
    //setup robot_fin_controller
    return 0;
}
