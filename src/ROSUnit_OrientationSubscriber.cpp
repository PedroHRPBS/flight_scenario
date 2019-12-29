#include "ROSUnit_OrientationSubscriber.hpp"
ROSUnit_OrientationSubscriber* ROSUnit_OrientationSubscriber::_instance_ptr = NULL;
Vector3DMessage ROSUnit_OrientationSubscriber::orientation_msg;

ROSUnit_OrientationSubscriber::ROSUnit_OrientationSubscriber(ros::NodeHandle& t_main_handler) : ROSUnit(t_main_handler)  {

    _sub_orientation = t_main_handler.subscribe("body_orientation", 10, callbackOrientation);
    _instance_ptr = this;

}

ROSUnit_OrientationSubscriber::~ROSUnit_OrientationSubscriber() {

}

void ROSUnit_OrientationSubscriber::callbackOrientation(const geometry_msgs::PointStamped& msg){

    Vector3D<float> tmp;
    tmp.x = msg.point.x;
    tmp.y = msg.point.y;
    tmp.z = msg.point.z;

    orientation_msg.setVector3DMessage(tmp);

    _instance_ptr->emit_message((DataMessage*) &orientation_msg); 

}

void ROSUnit_OrientationSubscriber::receive_msg_data(DataMessage* t_msg){

}