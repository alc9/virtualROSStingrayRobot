/*
*  Filename: robot_fin_state.cpp
*  Start date: 23/06/2022
*/
#include "robot_fin_state.h"

namespace wave_model
{

RobotFinState::RobotFinState(const ros::NodeHandle &nh):
    name_("robot_fin_state"),nh_(nh)
{
    if (!rosparam_shortcuts::get("wave_model",nh_,"robot_fin_state_mode",control_mode_)){
            rosparam_shortcuts::shutdownIfError(name_,true);
    }
    std::string endPoint;
    if ((!rosparam_shortcuts::get("wave_model",nh_,"right_fin",endPoint))){
            ROS_ERROR_STREAM("Action server namespace must be provided");
            rosparam_shortcuts::shutdownIfError(name_,true);
    }
    action_client_fins_ = new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>(nh_,endPoint,false);
    action_client_fins_->waitForServer();
    ROS_INFO_STREAM("Server is up for RobotFinState action client");
    
    std::vector<std::string> joints;
    if ((!rosparam_shortcuts::get("hardware_interface",nh_,"joints",joints))){
            ROS_ERROR_STREAM("Action server namespace must be provided");
            rosparam_shortcuts::shutdownIfError(name_,true);
    }
    //set base types for action_goal_msg
    action_goal_msg_.trajectory.joint_names=joints;
    freq_left_=1.4;
    freq_right_=1.4;
    //joint_names_=joints;
    //action_goal_msg_.path_tolerance=
}
RobotFinState::~RobotFinState(){
    delete action_client_fins_;
}
void RobotFinState::setPhaseDifference(){
    phase_diff_right_=freq_right_*2.0*M_PI/(action_goal_msg_.trajectory.joint_names.size()/2);
    phase_diff_left_=freq_left_*2.0*M_PI/(action_goal_msg_.trajectory.joint_names.size()/2);
}

void RobotFinState::read(){}
double RobotFinState::waveGeneratorFactory(const std::string& jointPattern,int index){
    if (jointPattern.rfind("R",0==0)){
        return(waveGenerator(freq_right_,smooth_time_right_,phase_diff_right_,index));
    }
    else if (jointPattern.rfind("L",0)==0){
        return(waveGenerator(freq_left_,smooth_time_right_,phase_diff_left_,index));
    }
    ROS_ERROR_STREAM("Error in config.yaml, should start with either R or L");
    throw;
}
void RobotFinState::setActionGoalMsg(){
    for (size_t joint=0; joint!=action_goal_msg_.trajectory.joint_names.size(); joint++){
            //fill JointTrajectoryPointMSG
            //want to go over a couple of points
            action_goal_msg_.trajectory.points[joint].positions[0]=this->waveGeneratorFactory(action_goal_msg_.trajectory.joint_names[joint],static_cast<int> (joint));
    }
}
void RobotFinState::write(){
    //write message to control joints
    this->setActionGoalMsg();
    action_client_fins_->sendGoal(action_goal_msg_);
}
//void RobotFinState::init(ros::NodeHandle &nh){}
void RobotFinState::PIDController(){}
void RobotFinState::controlLoop(){
    switch(control_mode_){
        case CONTROLMODE::frequency:
        {
            this->write();
            this->read();
            break;
        }
        case CONTROLMODE::goal:
            ROS_INFO_STREAM("goal mode not implemented");
            //this->PIDController();
            this->write();
            this->read();
            break;
        }
    }
} //namespace