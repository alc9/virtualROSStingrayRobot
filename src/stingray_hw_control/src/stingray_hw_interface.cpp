/*
* Filename: stingray_hw_interface.cpp
* Description: load URDF, perform control
* Author: Alex Cunningham
* Start date: 14/06/2022
*/
#include "stingray_hw_interface.h"
#include "wave.h"
StingrayHWInterface::StingrayHWInterface(ros::NodeHandle &nh,urdf::Model *urdf_model)
    : ros_control_boilerplate::GenericHWInterface(nh,urdf_model)
{
    nh_=new ros::NodeHandle();
    this->init();
    initStingrayHWInterface();
    //get joint angle IDs
}

StingrayHWInterface::~StingrayHWInterface(){
    delete nh_;
}

void StingrayHWInterface::initStingrayHWInterface(void) noexcept{
    //TODO: initialize wave, control, frequency ...
    ROS_INFO_STREAM("Initializing StingrayHWInterface");
    upwards_=true;
    control_param_lock_=false;
    //get IDs for base joint for each actuator (mimic actuator)
    auto jointIdNames=std::array<std::string,1>{"R1_base_link_to_2nd_link"}; //"L1_base_link_to_2nd_link"
    std::for_each(jointIdNames.begin(),jointIdNames.end(),[this](std::string& str) mutable{
        auto pos = std::find(this->joint_names_.begin(),this->joint_names_.end(),str);
        if (pos==joint_names_.end()){
            std::cerr<<boost::format("jointIDName %1% is missing") % str;
            std::cerr<<"exiting...";
            ros::shutdown();
        }
        else{
            actuator_ids_[str.substr(0,2)]=pos-joint_names_.begin();
        }
        });
    //definitiion in config.yaml + .launch
    actuator_pubs_right_={nh_->advertise<std_msgs::Float64>("/stingray/actuator1/command",0),
    nh_->advertise<std_msgs::Float64>("/stingray/actuator2/command",0),
    nh_->advertise<std_msgs::Float64>("/stingray/actuator3/command",0),
    nh_->advertise<std_msgs::Float64>("/stingray/actuator4/command",0),
    nh_->advertise<std_msgs::Float64>("/stingray/actuator5/command",0)};

    actuator_pubs_left_={nh_->advertise<std_msgs::Float64>("/stingray/actuator6/command",0),
    nh_->advertise<std_msgs::Float64>("/stingray/actuator7/command",0),
    nh_->advertise<std_msgs::Float64>("/stingray/actuator8/command",0),
    nh_->advertise<std_msgs::Float64>("/stingray/actuator9/command",0),
    nh_->advertise<std_msgs::Float64>("/stingray/actuator10/command",0)};

    //set states used for producing wave
    upwards_=true;
    f_right_=1.0;
    f_right_prev_=f_right_;
    smooth_time_right_=0.0;
    phaseDif_right_=f_right_*2*M_PI/(actuator_ids_.size()/2);
    //move to initialize wave position
    //TODO: both right and left/ plus mesh
    writeJointPositionsRight();
    ROS_INFO_STREAM("StingrayHWInterface initialized");
}

//Read joint positions, inform if angles met
void StingrayHWInterface::read(ros::Duration &elapsed_time){
    //goal reached -> what frequency, position and velocity
    //read registers import subscribed parameters
    if (((joint_position_[actuator_ids_["R1"]]>=joint_angle_goal_*0.99) ^ upwards_)){
        //lock access to control params from subscribers
        control_param_lock_=true;
    }
    return;
}


void StingrayHWInterface::enforceLimits(ros::Duration& period){
    //enforce position and velocity
    pos_jnt_sat_interface_.enforceLimits(period);
}
//TODO: error writing joint position
void StingrayHWInterface::writeJointPositionsRight(){
    //iterate over joint publishers
    for (auto actuatorPubIt=actuator_pubs_right_.begin(); actuatorPubIt!=actuator_pubs_right_.end();actuatorPubIt++){
        joint_angle_msg_right_.data=waveGenerator(f_right_,smooth_time_right_,phaseDif_right_, actuatorPubIt - actuator_pubs_right_.begin());
        std::cout<<"writing joint position: "<<joint_angle_msg_right_.data<<std::endl;
        if (actuatorPubIt==actuator_pubs_right_.begin()){
            upwards_=joint_angle_msg_right_.data>joint_position_[actuator_ids_["R1"]]?true:false;
        }
        actuatorPubIt->publish(joint_angle_msg_right_);
    }
}

void StingrayHWInterface::writeJointPositionsLeft(){
    //iterate over joint publishers
    for (auto actuatorPubIt=actuator_pubs_left_.begin(); actuatorPubIt!=actuator_pubs_left_.end();actuatorPubIt++){
        joint_angle_msg_left_.data=waveGenerator(f_left_,smooth_time_left_,phaseDif_left_, actuatorPubIt - actuator_pubs_left_.begin());
        if (actuatorPubIt==actuator_pubs_left_.begin()){
            upwards_=joint_angle_msg_left_.data>joint_position_[actuator_ids_["L1"]]?true:false;
        }
        actuatorPubIt->publish(joint_angle_msg_left_);
    }
}

//This method sets control params,
// @TODO update for control eqn ; 
void StingrayHWInterface::setControlParams(){
    smooth_time_right_=(asin(2*M_PI*f_right_prev_*smooth_time_right_)/(2*M_PI*f_right_))+delay_time_;
    std::cout<<"smooth time is: "<<smooth_time_right_<<std::endl;
    //time_+=delay_time_;
}

void StingrayHWInterface::write(ros::Duration &elapsed_time){
    std::cout<<"write"<<std::endl;
    enforceLimits(elapsed_time);
    //control_param_lock_ == true if reached goal angle
    if (!control_param_lock_){
        //increment time
        return;
    }
    //actuator has reached goal angle
    setControlParams();
    //TODO: set joint positions and wave - on different threads then wait
    //TODO: e.g threads(meshFinLeft,meshFinRight) then threads(writeJointPositionsLeft,writeJointPositionsRight)
    writeJointPositionsRight();
    control_param_lock_=false;
    //if goal position reached then call waveGenerator
    //set joint positions and wave - on different threads then wait

}

