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
    if ((!rosparam_shortcuts::get("wave_model",nh_,"action_server_endpoint",endPoint))){
            ROS_ERROR_STREAM("Action server namespace must be provided");
            rosparam_shortcuts::shutdownIfError(name_,true);
    }
    //@TODO: this class is configurable depending on .yaml file -> might be better to develop a factory that uses a builder 
    action_client_fins_ = new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>(nh_,endPoint,false);
    action_client_fins_->waitForServer();
    ROS_INFO_STREAM("Server is up for RobotFinState action client");

    //get joint names used in action.goal 
    std::vector<std::string> joints;
    //rosparams serve joints on left or right - whether or not this is performed is determined in 
    //robot_fin_controller via param=right_fin/left_fin
    if ((!rosparam_shortcuts::get("hardware_interface",nh_,"joints",joints))){
            ROS_ERROR_STREAM("Action server namespace must be provided");
            rosparam_shortcuts::shutdownIfError(name_,true);
    }
    //set base types for action_goal.msg
    action_goal_msg_.trajectory.joint_names=joints;
    freq_left_=1.4;
    freq_right_=1.4;
    //check format of joint messages -> right,right,right,left,left,left or mirrored
    std::string format_check_str= std::accumulate(joints.begin(),joints.end(),std::string(" "));
    //check pattern against regular expression - enforce yaml format for hardware_interface joints
    if (!std::regex_match(format_check_str,std::regex("(^[L]|^[R])[^\\s]{1,}(\\s|$)(\1[^\\s]{0,}\\s){0,}(([L|R])[^\\s]{1,}(\\s|$)|$)((([\5][^\\s]{1,}(\\s|$)){1,})|$)$"))){
        ROS_ERROR_STREAM("incorrect parameter format - should be a series of right joints followed by a series of left joints indicated by first letter L or R");
        rosparam_shortcuts::shutdownIfError(name_,true);
    }
    //check is_stride_left, if true then applying stide means joint names ordered [rightfinJoints,leftfinJoints]
    is_stride_left_=(char)format_check_str[0]=='R'?true:false;
    //determine stride
    auto tmp_stride_=std::find_if(joints.end(),joints.begin(),[this](const std::string& str){
        if(str[0]==is_stride_left_?'L':'R'){
            return true;
        }
        return false;
    });
    if (tmp_stride_!=joints.end()){
        stride_ = new int(std::distance(joints.end(),tmp_stride_));
        //stride_=&std::distance(joints.end(),tmp_stride_);
    }
    //joint_names_=joints;
    //action_goal_msg_.path_tolerance=
}
RobotFinState::~RobotFinState(){
    delete action_client_fins_;
    delete stride_;
}
void RobotFinState::setPhaseDifference(){
    phase_diff_right_=freq_right_*2.0*M_PI/(action_goal_msg_.trajectory.joint_names.size()/2);
    phase_diff_left_=freq_left_*2.0*M_PI/(action_goal_msg_.trajectory.joint_names.size()/2);
}

void RobotFinState::read(){}

double RobotFinState::waveGeneratorFactory(int index){
    //stride indicates whether writing to left of right fin joint
    //is_stride_left_ indicates order given in yaml file [left_joints,right_joints] or [right_joints,left_joints]
    if (stride_){
        if (index<*stride_ && is_stride_left_){
            return(waveGenerator(freq_right_,smooth_time_right_,phase_diff_right_,index));
        }
        else if (index>*stride_ && is_stride_left_){
            return(waveGenerator(freq_left_,smooth_time_right_,phase_diff_left_,index));
        }
        else if (index<*stride_){
            return(waveGenerator(freq_left_,smooth_time_right_,phase_diff_left_,index));
        }
        else if (index>*stride_){
            return(waveGenerator(freq_right_,smooth_time_right_,phase_diff_right_,index));
        }
    }
    else{
        if (is_stride_left_){
            return(waveGenerator(freq_right_,smooth_time_right_,phase_diff_right_,index));
        }
        return(waveGenerator(freq_left_,smooth_time_left_,phase_diff_left_,index));
    }
    ROS_ERROR_STREAM("Error in config.yaml, should start with either R or L");
    throw;
}

void RobotFinState::setActionGoalMsg(){
        for (size_t joint=0; joint!=action_goal_msg_.trajectory.joint_names.size(); joint++){
            //fill JointTrajectoryPointMSG
            //want to go over a couple of points
            action_goal_msg_.trajectory.points[joint].positions[0]=this->waveGeneratorFactory(static_cast<int> (joint));
        }
}

void RobotFinState::write(){
    //write message to control joints
    this->setActionGoalMsg();
    action_client_fins_->sendGoal(action_goal_msg_, boost::bind(&RobotFinState::doneCb,this,_1));
    action_client_fins_->waitForResult();
}
void RobotFinState::doneCb(const actionlib::SimpleClientGoalState &state){
    clientGoalState=state.state_;
}

bool RobotFinState::shouldCancelRun(){
    //if server disconnected then try to connect
    action_client_fins_->waitForServer(ros::Duration(10));
    if (action_client_fins_->isServerConnected()){
        ROS_INFO_STREAM("server non-responsive, exiting control loop...");
        return true;
    }
    return false;
    //if SUCCEEDED false determine whether to cancel or not
}
void RobotFinState::run(){
    while(ros::ok()){
        if (clientGoalState == actionlib::SimpleClientGoalState::StateEnum::SUCCEEDED && action_client_fins_->isServerConnected()){
        PIDController();
        update();
        }
        else if (shouldCancelRun()){
            return;
        }
    }
}
//void RobotFinState::init(ros::NodeHandle &nh){}
void RobotFinState::PIDController(){}
void RobotFinState::update(){
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