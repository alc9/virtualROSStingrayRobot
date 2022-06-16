/*
* nada
*/

#include "stingray_hw_interface.h"
#include <ros/ros.h>
#include <ros_control_boilerplate/generic_hw_control_loop.h>
class StingrayHWInterface;

int main(int argc,char** argv){
    ros::init(argc,argv,"control_node");
    ros::NodeHandle nh;
    std::shared_ptr<urdf::Model> urdfModel (new urdf::Model());
    //urdf::Model* urdfModel;
    //urdfModel = new urdf::Model();
    urdfModel->initParam("/home/alex/projects/stingRay/virtualStingray/src/virtualStingray_description/urdf/robot_simplified.urdf");
    std::shared_ptr<StingrayHWInterface> interface (new StingrayHWInterface(nh,urdfModel.get()));
    interface ->init();
    ros_control_boilerplate::GenericHWControlLoop control_loop(nh,interface);
    ros::waitForShutdown();
    return 0;
}
