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

//Control related functionality: control system eqn, joint motion control and fin wave joint generation
//@param nh - node handle 
//@param urdf_model - urdf model for robot
class StingrayHWInterface : public ros_control_boilerplate::GenericHWInterface{
        StingrayHWInterface(const StingrayHWInterface& s) = delete;
        auto operator=(const StingrayHWInterface& s)->StingrayHWInterface& = delete;
    protected:
        ros::NodeHandle* nh_=nullptr;
        ros::Subscriber* joint_pos_sub_=nullptr;
        ros::Subscriber* 
    public:
        //manage load urdf model via init
        StingrayHWInterface(ros::NodeHandle &nh,urdf::Model* urdf_model);
        
        virtual ~StingrayHWInterface();
        
        //Check if goal position is reached, if true then notify write
        virtual void read(ros::Duration &elapsed_time);

        virtual void write(ros::Duration &elapsed_time);

        virtual void enforceLimits(ros::Duration &period); 
    private:
        void initializeSubscribers(void) noexcept;
        void init(void) noexcept; 
        //TODO: pointers to publishers?
        std::array<ros::Publisher,2> actuator_pubs_;
        //offset for actuator positions - left_fin + index
        std_msgs::Float32 joint_angle_msg_;
        //
        float joint_angle_goal_;
        //current goal joint angle
        float joint_angle_;
        //joints are increasing to goal angle
        bool upwards_;
        //limit write to control_param_lock_ for subscribers
        bool control_param_lock_;
};
