/*
* Filename: stingray_hw_interface.h
* Description: ROS hardware interface provides an interface which loads URDF, 
* sets joint positions and wave mesh
* Author: Alex Cunningham
* Start date: 13/6/2022
*/
#pragma once
#include <ros_control_boilerplate/generic_hw_interface.h>
#include <ros/ros.h>
#include <array>
#include <std_msgs/Float64.h>
#include <unordered_map>
#include <boost/format.hpp>
#include <string>
#include <iostream>
#include <algorithm>
#include <thread>
#include <rosparam_shortcuts/rosparam_shortcuts.h>
#include <tuple>

typedef std::unordered_map<std::string,int> ActuatorIds;
//control_mode set in yaml (parameter server)
//id=0 pos only, id=1 pos and wave
struct ControlMode { unsigned int pos,posWave; };

//Control related functionality: control system eqn, joint motion control and fin wave joint generation
//@param nh - node handle 
//@param urdf_model - urdf model for robot
class StingrayHWInterface : public ros_control_boilerplate::GenericHWInterface{
        StingrayHWInterface(const StingrayHWInterface& s) = delete;
        auto operator=(const StingrayHWInterface& s)->StingrayHWInterface& = delete;
    protected:
        int control_mode_;
        //ros::NodeHandle* nh_=nullptr;
    public:
        //manage load urdf model via init
        StingrayHWInterface(ros::NodeHandle &nh,urdf::Model* urdf_model=NULL);
        
        virtual ~StingrayHWInterface();   
        //Check if goal position is reached, if true then notify write
        virtual void read(ros::Duration &elapsed_time){}

        virtual void write(ros::Duration &elapsed_time);

        virtual void enforceLimits(ros::Duration &period); 
    private:

        /**
         * @brief Factory for setting initial control params
         * @param noexcept
         * @return (void)
         */
        void initStingrayHWInterface(void) noexcept;
        void setWaveLeft();
        void setWaveRight();
        //threads for making membrane - left and right fins
        std::thread * thread_left_fin_= nullptr;
        std::thread * thread_right_fin_= nullptr;
        //map of actuatorID and name
        ActuatorIds actuator_ids_;
        //limit write to control_param_lock_ for subscribers
        bool control_param_lock_;
        //delay between adjusting control parameters
        double delay_time_=0.005;
        //smooth transition time adjustment
        double smooth_time_right_=0.0;
        double smooth_time_left_=0.0;
        //right hand time
        double f_right_;
        double phaseDif_right_;
        //left hand side
        double f_left_prev_;
        double f_left_;
        double phaseDif_left_;
};
