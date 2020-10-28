#pragma once
#include "common_srv/ROSUnit.hpp"
#include "PositionMsg.hpp"
#include <geometry_msgs/Point.h>

class ROSUnit_PositionSubscriber : public ROSUnit {

private:  
    static Port* _output_port_0;
    ros::Subscriber _sub_position;
    static ROSUnit_PositionSubscriber* _instance_ptr;
    static PositionMsg position_msg; 
    static void callbackPosition(const geometry_msgs::Point& msg);
       
public:
    enum ports_id {OP_0};
    void process(DataMessage* t_msg, Port* t_port) {};
    ROSUnit_PositionSubscriber(ros::NodeHandle&);
    ~ROSUnit_PositionSubscriber();
};