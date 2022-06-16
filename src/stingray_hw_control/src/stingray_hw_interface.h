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
#include <std_msgs/Float64.h>
#include <unordered_map>
#include <boost/format.hpp>
#include <string>
#include <iostream>
#include <algorithm>


typedef std::unordered_map<std::string,int> ActuatorIds;
//data class for actuator base links
//struct ActuatorIds{ unsigned int R1,R2,R3,R4,R5,L1,L2,L3,L4,L5; };
//Control related functionality: control system eqn, joint motion control and fin wave joint generation
//@param nh - node handle 
//@param urdf_model - urdf model for robot
class StingrayHWInterface : public ros_control_boilerplate::GenericHWInterface{
        StingrayHWInterface(const StingrayHWInterface& s) = delete;
        auto operator=(const StingrayHWInterface& s)->StingrayHWInterface& = delete;
    protected:
        ros::NodeHandle* nh_=nullptr;
    public:
        //manage load urdf model via init
        StingrayHWInterface(ros::NodeHandle &nh,urdf::Model* urdf_model);
        
        virtual ~StingrayHWInterface();
        
        //Check if goal position is reached, if true then notify write
        virtual void read(ros::Duration &elapsed_time);

        virtual void write(ros::Duration &elapsed_time);

        virtual void enforceLimits(ros::Duration &period); 
    private:
        void initStingrayHWInterface(void) noexcept;
        void writeJointPositionsLeft();
        void writeJointPositionsRight();
        void setControlParams();
        //TODO: pointers to publishers?
        std::array<ros::Publisher,5> actuator_pubs_right_;
        std::array<ros::Publisher,5> actuator_pubs_left_;
        //used for assigning double values
        //two copies to avoid race condition
        std_msgs::Float64 joint_angle_msg_left_;
        std_msgs::Float64 joint_angle_msg_right_;
        ActuatorIds actuator_ids_;
        //current goal for joint angle
        double joint_angle_goal_;
        //joints are increasing to goal angle - +ve frequency = upwards
        bool upwards_;
        //limit write to control_param_lock_ for subscribers
        bool control_param_lock_;
        
        //delay between adjusting control parameters
        double delay_time_=0.005;
        //smooth transition time adjustment
        double smooth_time_right_=0.0;
        double smooth_time_left_=0.0;
        //right hand side
        //no point redoing everything if f_right_ is approx f_right_prev
        double f_right_prev_;
        double f_right_;
        double phaseDif_right_;
        double joint_angle_goal_right_;
        //left hand side
        double f_left_prev_;
        double f_left_;
        double phaseDif_left_;
        double joint_angle_goal_left_;

};
