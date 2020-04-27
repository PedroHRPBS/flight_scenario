#pragma once
#include "common_srv/MsgEmitter.hpp"
#include "common_srv/MsgReceiver.hpp"
#include "common_srv/ROSUnit.hpp"

class FlightElement : public MsgEmitter, public MsgReceiver{

public:

    virtual void perform() = 0;
    virtual void receiveMsgData(DataMessage* t_msg) = 0;

    FlightElement();
    ~FlightElement();
};