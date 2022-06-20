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
    //nh_=new ros::NodeHandle();
    this->init();
    initStingrayHWInterface();
    //get joint angle IDs
}

StingrayHWInterface::~StingrayHWInterface(){
    //delete nh_;
    if(thread_left_fin_->joinable()){
        thread_left_fin_->join();
    }
    if(thread_right_fin_->joinable()){
        thread_right_fin_->join();        
    }
    delete thread_left_fin_;
    delete thread_right_fin_;
}

void StingrayHWInterface::initStingrayHWInterface(void) noexcept{
    //@TODO: initialize wave, control, frequency ...
    //@TODO: add some error handling
    // https://rules.sonarsource.com/cpp/tag/bad-practice/RSPEC-3743
    ROS_INFO_STREAM("Initializing StingrayHWInterface");
    control_param_lock_=false;
    std::size_t error=0;
    //TODO: set control_mode
    error+=rosparam_shortcuts::get(name_,nh_,"control_mode", control_mode_);
    if (error){
    ROS_WARN_STREAM_NAMED(name_, "require control_mode to be set");
    ROS_WARN_STREAM_NAMED(name_, "  0: only pos, 1: pos and wave membrane");
  }
  rosparam_shortcuts::shutdownIfError(name_, error);
    //get IDs for base joint for each actuator (mimic actuator)
    //will need this for calculating left/right membranes
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

    f_left_=1.4;
    f_right_=1.4;

    //@TODO this will be actuator_ids_.size()/2 when left fin is implemented
    phaseDif_right_=f_right_*2.0*M_PI/(actuator_ids_.size());
    phaseDif_left_=phaseDif_right_;
    ROS_INFO_STREAM("StingrayHWInterface initialized");
}

void StingrayHWInterface::enforceLimits(ros::Duration& period){
    //enforce position and velocity
    pos_jnt_sat_interface_.enforceLimits(period);
}

/*
void StingrayHWInterface::writeJointPositionsRight(){
    //iterate over joint publishers
    for (auto actuatorPubIt=actuator_pubs_right_.begin(); actuatorPubIt!=actuator_pubs_right_.end();actuatorPubIt++){
        //std::cout<<"f_right_= " << f_right_ << "smooth_time_= " << smooth_time_right_ << "phaseDif_right_= " << phaseDif_right_ << std::endl; 
        joint_angle_msg_right_.data=waveGenerator(f_right_,smooth_time_right_,phaseDif_right_,actuatorPubIt - actuator_pubs_right_.begin());
        //std::cout<<"writing joint position: "<<joint_angle_msg_right_.data<<std::endl;
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
    //std::cout<<"smooth time is: "<<smooth_time_right_<<std::endl;
    //time_+=delay_time_;
}
*/

/**
 * @brief Write joint positions and set membrane (left and right waves)
 * @param elapsed_time
 * @return (void)
 */
void StingrayHWInterface::write(ros::Duration &elapsed_time){
    enforceLimits(elapsed_time);
    //update joints and waves
    for (std::size_t joint_id = 0; joint_id < num_joints_; ++joint_id){
        //simple jointTrajectorycontroller -> only position atm
        joint_position_[joint_id] += joint_position_command_[joint_id];  

    }
}

