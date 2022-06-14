/*
* Filename: stingray_hw_interface.cpp
* Description: load URDF, perform control
* Author: Alex Cunningham
* Start date: 14/06/2022
*/
#include "stingray_hw_interface.h"
#include <std_msgs/Float32.h>
#include <iostream>
#include "wave.h"
StingrayHWInterface::StingrayHWInterface(ros::NodeHandle &nh,urdf::Model *urdf_model)
    : ros_control_boilerplate::GenericHWInterface(nh,urdf_model)
{
    nh_=new ros::NodeHandle();
    joint_angle_=std_msgs::Float32();
    actuator_pubs_={nh_->advertise<std_msgs::Float32>("/stingray/actuator1/command",0),
    nh_->advertise<std_msgs::Float32>("/stingray/joint_manipulator",0)};
}

StingrayHWInterface::~StingrayHWInterface(){
    //delete nh_urdf_;
    delete nh_;
}
void StingrayHWInterface::read(ros::Duration &elapsed_time){}
void StingrayHWInterface::enforceLimits(ros::Duration& period){}

void StingrayHWInterface::write(ros::Duration &elapsed_time){
    //TODO: write position
    //waveGenerator(); 
    //for (auto i : waveGenerator){
    //    std::cout<<i<<'\n'; 
    //    }
}

