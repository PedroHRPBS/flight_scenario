#pragma once
#include "common_srv/ROSUnit.hpp"
#include "InfoMsg.hpp"
#include <flight_controller/Info.h>

class ROSUnit_InfoSubscriber : public ROSUnit {

private:  
    static Port* _output_port_0;
    ros::Subscriber _sub_info;
    static ROSUnit_InfoSubscriber* _instance_ptr;
    static InfoMsg info_msg; 
    static void callbackInfo(const flight_controller::Info& msg);
       
public:
    enum ports_id {OP_0};
    void process(DataMessage* t_msg, Port* t_port) {};
    ROSUnit_InfoSubscriber(ros::NodeHandle&);
    ~ROSUnit_InfoSubscriber();
};