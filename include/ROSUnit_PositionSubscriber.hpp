#pragma once
#include "ROSUnit.hpp"
#include "PositionMsg.hpp"
#include <geometry_msgs/PointStamped.h>

class ROSUnit_PositionSubscriber : public ROSUnit {

private:  
    ros::Subscriber _sub_position;
    static ROSUnit_PositionSubscriber* _instance_ptr;
    static PositionMsg position_msg; 
    static void callbackPosition(const geometry_msgs::PointStamped& msg);
       
public:

    void receive_msg_data(DataMessage*);
    ROSUnit_PositionSubscriber(ros::NodeHandle&);
    ~ROSUnit_PositionSubscriber();
};