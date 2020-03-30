#pragma once
#include "ROSUnit.hpp"
#include "InfoMsg.hpp"
#include <flight_controller/Info.h>

class ROSUnit_InfoSubscriber : public ROSUnit {

private:  
    ros::Subscriber _sub_info;
    static ROSUnit_InfoSubscriber* _instance_ptr;
    static InfoMsg info_msg; 
    static void callbackInfo(const flight_controller::Info& msg);
       
public:

    void receiveMsgData(DataMessage*);
    ROSUnit_InfoSubscriber(ros::NodeHandle&);
    ~ROSUnit_InfoSubscriber();
};