#include "ROSUnit_OrientationSubscriber.hpp"
ROSUnit_OrientationSubscriber* ROSUnit_OrientationSubscriber::_instance_ptr = NULL;
Vector3DMessage ROSUnit_OrientationSubscriber::orientation_msg;
Port* ROSUnit_OrientationSubscriber::_output_port_0 = new OutputPort(ports_id::OP_0, NULL);

ROSUnit_OrientationSubscriber::ROSUnit_OrientationSubscriber(ros::NodeHandle& t_main_handler) : ROSUnit(t_main_handler)  {

    _sub_orientation = t_main_handler.subscribe("uav_control/uav_orientation", 2, callbackOrientation);
    _instance_ptr = this;
    _ports = {_output_port_0};

}

ROSUnit_OrientationSubscriber::~ROSUnit_OrientationSubscriber() {

}

void ROSUnit_OrientationSubscriber::callbackOrientation(const geometry_msgs::Point& msg){

    Vector3D<float> tmp;
    tmp.x = msg.x;
    tmp.y = msg.y;
    tmp.z = msg.z;

    orientation_msg.setVector3DMessage(tmp);
    _output_port_0->receiveMsgData((DataMessage*) &orientation_msg); 
}