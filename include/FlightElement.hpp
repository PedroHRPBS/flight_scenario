#pragma once
#include "MsgEmitter.hpp"
#include "MsgReceiver.hpp"
#include "ROSUnit.hpp"

class FlightElement : public MsgEmitter, public MsgReceiver{

public:

    virtual void perform() = 0;
    virtual void receiveMsgData(DataMessage* t_msg) = 0;

    FlightElement();
    ~FlightElement();
};