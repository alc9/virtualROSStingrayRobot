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
    initStingrayHWInterface();
    //get joint angle IDs
}

StingrayHWInterface::~StingrayHWInterface(){
    //delete nh_urdf_;
    delete nh_;
}


void StingrayHWInterface::initStingrayHWInterface(void) noexcept{
    
    //TODO: initialize wave, control, frequency ...
    //get IDs for base joint for each actuator (mimic actuator)
    auto jointIdNames=std::array<std::string,5>{"R1_base_link_to_2nd_link"," "," "," ", " "};
    std::for_each(jointIdNames.begin(),jointIdNames.end(),[this](std::string& str) mutable{
        auto pos = std::find(this->joint_names_.begin(),this->joint_names_.end(),str);
        if (pos==joint_names_.end()){
            std::cerr<<boost::format("jointIDName %1% is missing") % 1;
            std::cerr<<"exiting...";
            ros::shutdown();
        }
        else{
            actuator_ids_[str.substr(0,2)]=pos-joint_names_.begin();
        }
        });
    //definitiion in config.yaml + .launch
    actuator_pubs_={nh_->advertise<std_msgs::Float64>("/stingray/actuator1/command",0),
    nh_->advertise<std_msgs::Float64>("/stingray/joint_manipulator",0)};
}

void StingrayHWInterface::read(ros::Duration &elapsed_time){
    //goal reached -> what frequency, position and velocity
    //read registers import subscribed parameters
    
    if (((joint_position_[actuator_ids_["R1"]]>=joint_angle_goal_*0.99) ^ upwards_) || control_param_lock_){
        //lock access to control params from subscribers
        control_param_lock_=true;
    }
    return;
}
void StingrayHWInterface::enforceLimits(ros::Duration& period){
    //enforce position and velocity
    pos_jnt_sat_interface_.enforceLimits(period);
}

void StingrayHWInterface::write(ros::Duration &elapsed_time){
    //TODO: write position
    //waveGenerator(); 
    //for (auto i : waveGenerator){
    //    std::cout<<i<<'\n'; 
    //    }
    enforceLimits(elapsed_time);
    if (!control_param_lock_){
        return;
    }
    control_param_lock_=false;
    //if goal position reached then call waveGenerator
    //set joint positions and wave - on different threads then wait

}

