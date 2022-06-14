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
    joint_angle_=std_msgs::Float32();
}

StingrayHWInterface::read(ros::Duration &elapsed_time){}

StingrayHWInterface::write(ros::Duration &elapsed_time){
    
}

