/*
*  Filename: robot_fin_state.h
*  Author: Alex Cunningham
*  Description: This class performs similar roles to pr2_mechanism_model::RobotState except
*  except RobotState::model_ only contains hardware interface
*  Start date: 21/06/2022
*/    
#pragma once
#include <ros/ros.h> 
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/JointTrajectoryGoal.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include "wave.h"
//messages used to build FollowJointTrajectory

//std
#include <vector>
#include <string>
#include <algorithm>
//0=frequency mode(no control), 1=goal navigation mode(perform control)
enum CONTROLMODE { frequency, goal };   

namespace wave_model{
    class RobotFinState{
            public:
                RobotFinState(const ros::NodeHandle &nh);
                virtual ~RobotFinState();
                /**
                 * @brief Read in control variables for robot, access for controller via protected attributes
                 * @param
                 * @return 
                 */
                virtual void read();
                /**
                 * @brief Write control variables to joint controller via actions
                 * @param
                 * @return 
                 */
                virtual void write();
                //virtual void init(ros::NodeHandle &nh);
                virtual void PIDController();
                virtual void controlLoop();
            protected:
                ros::NodeHandle nh_;
                const std::string name_;
                int control_mode_;
            private:
                double waveGeneratorFactory(const std::string& jointPattern, int index);
                void setActionGoalMsg();
                void setPhaseDifference();
                const int * const stride_=nullptr;
                bool is_stride_left_;
                double freq_left_;
                double freq_right_;
                //@TODO: add change smooth_time changes
                double smooth_time_right_;
                double smooth_time_left_;
                double phase_diff_right_;
                double phase_diff_left_;
                //control variables read in and used in ouptut via actions for joint control
                float position_x_;
                actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> * action_client_fins_=nullptr;
                control_msgs::FollowJointTrajectoryGoal action_goal_msg_;

    };
}