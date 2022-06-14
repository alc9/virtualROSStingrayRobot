/*
* Filename: stingray_hw_interface.cpp
* Description: load URDF, perform control
* Author: Alex Cunningham
* Start date: 14/06/2022
*/
#include "include/stingray_hw_interface.h"
#include <std_msgs/Float32.h>

class StingrayHWInterface;

StingrayHWInterface::StingrayHWInterface(ros::NodeHandle &nh)
    : ros_control_boilerplate::GenericHWInterface(nh)
{
    nh_urdf_=new ros::NodeHandle("/stingray/urdf");
    this->loadURDF(nh_urdf_,"stingray");
    nh_=new ros::NodeHandle()
    joint_angle_=std_msgs::Float32();
    actuator_pubs_={new
    nh_.advertise<std_msgs::Float32>("/stingray/actuator1/command"), new
    nh_advertise<std_msgs::Float32("/stingray/joint_manipulator")>};
}

StingrayHWInterface::~StingrayHWInterface(){
    delete nh_urdf_;
    delete nh_;
    for (auto i : actuator_pubs_ ){
        delete i;
    }
}
StingrayHWInterface::read(ros::Duration &elapsed_time){}

StingrayHWInterface::write(ros::Duration &elapsed_time){
    
}

