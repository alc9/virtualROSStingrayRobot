/*
* Filename: stingray_hw_interface.h
* Description: provides an interface which loads URDF, performs control based
* on required waveform, sets joint positions
* Author: Alex Cunningham
* Start date: 13/6/2022
*/
#pragma once
#include <ros_control_boilerplate/generic_hw_interface.h>
#include <array>
#include <std_msgs/Float32.h>

class StingrayHWInterface : public ros_control_boilerplate::GenericHWInterface{
        StingrayHWInterface(const StingrayHWInterface& s) = delete;
        auto operator=(const StingrayHWInterface& s)->StingrayHWInterface& = delete;
    public:
        //manage load urdf model via init
        StingrayHWInterface(ros::NodeHandle &nh,urdf::Model* urdf_model);
        virtual ~StingrayHWInterface();
        //overwrite base class members
        virtual void read(ros::Duration &elapsed_time);

        virtual void write(ros::Duration &elapsed_time);

        virtual void enforceLimits(ros::Duration &period); 
    private:
        //set actuator positions
        //pointer to publisher?
        std::array<ros::Publisher,2> actuator_pubs_;
        //offset for actuator positions - left_fin + index
        ros::NodeHandle* nh_=nullptr;
        std_msgs::Float32 joint_angle_;
};
