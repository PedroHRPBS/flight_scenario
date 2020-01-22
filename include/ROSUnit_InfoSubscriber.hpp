#pragma once
#include "ROSUnit.hpp"
#include "InfoMsg.hpp"
#include <positioning_system/Info.h>

class ROSUnit_InfoSubscriber : public ROSUnit {

private:  
    ros::Subscriber _sub_info;
    static ROSUnit_InfoSubscriber* _instance_ptr;
    static InfoMsg info_msg; 
    static void callbackInfo(const positioning_system::Info& msg);
       
public:

    void receive_msg_data(DataMessage*);
    ROSUnit_InfoSubscriber(ros::NodeHandle&);
    ~ROSUnit_InfoSubscriber();
};