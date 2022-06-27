/*
*  Filename: wave_frequency_controller.cpp
*  Description: ROS controller listens to controller manager and stops when needed 
*  controllers(hw interface=stingray_hw_interface)
*  Author: Alex Cunningham
*  Start date: 21/6/2022
*/
/*
#pragma once
#include <ros/ros.h>
#include "robot_fin_state.h"
//id=0 position (move to a given waypoint), id=1 move at a given frequency, id=2 move in a straight line
struct ControlMode {int position, frequency, straight;};
//
namespace wave_controller_interface{
    class RobotFinController: public RobotFinState{
        public:
            RobotFinController(ros::NodeHandle &nh);
            ~RobotFinController();
            virtual bool init(ros::NodeHandle &nh,wave_model::RobotFinState* model);
            virtual void stopping();
        private:
            //Model contains hardware interface
            wave_model::RobotFinState* model_;
            int control_mode_;
    };
}
*/