/*
* Filename stingray_hw_interface.h
* Description: provides an interface which loads URDF, performs control based
* on required waveform, sets joint positions
* Author: Alex Cunningham
* Start date: 13/6/2022
*/
#pragma once
#include <ros_control_boilerplate/generic_hw_interface.h>
#include <array>

class StingrayHWInterface : public ros_control_boilerplate::GenericHWInterface{
    StingrayHWInterface(const StingrayHWInterface& s)=delete;
    auto operator=(const StingrayHWInterface& s)=delete;
    public:
        //manage load urdf model via init
        StingrayHWInterface(ros::NodeHandle &nh);
    
        //overwrite base class members
        virtual void read(ros::Duration &elapsed_time);

        virtual void write(ros::Duration &elapsed_time);

        virtual void enforceLimits(ros::Duration &period) 
    private:
        //set actuator positions
        //TODO: joint_group_velocity_controller?
        std::array<ros::Publisher,10> actuator_pubs_;
        //offset for actuator positions - left_fin + index
        const enum actuator_offset_{"left_fin"=0,"right_fin"=5};
};
