#pragma once
#include "MsgEmitter.hpp"
#include "MsgReceiver.hpp"
#include "ROSUnit.hpp"

class FlightElement : public ROSUnit{

public:

    virtual void perform() = 0;
    virtual void receive_msg_data(DataMessage* t_msg) = 0;

    FlightElement();
    FlightElement(ros::NodeHandle&);
    ~FlightElement();
};