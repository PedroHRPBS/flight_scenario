#include "ROSUnit_ControlOutputSubscriber.hpp"

ROSUnit_ControlOutputSubscriber* ROSUnit_ControlOutputSubscriber::_instance_ptr = nullptr;

Port* ROSUnit_ControlOutputSubscriber::_output_port_0 = new OutputPort(ports_id::OP_0, NULL);

ROSUnit_ControlOutputSubscriber::ROSUnit_ControlOutputSubscriber(ros::NodeHandle& t_main_handler) : ROSUnit(t_main_handler) {

    _sub_controloutput = t_main_handler.subscribe("control_system_output", 2, callbackControlOutput);
    _instance_ptr = this;
    _ports = {_output_port_0};
}

ROSUnit_ControlOutputSubscriber::~ROSUnit_ControlOutputSubscriber() {

}

void ROSUnit_ControlOutputSubscriber::callbackControlOutput(const std_msgs::Float64MultiArray& msg){

    VectorDoubleMsg _controloutput_msg;
    _controloutput_msg.data = msg.data;
    _output_port_0->receiveMsgData((DataMessage*) &_controloutput_msg);
}