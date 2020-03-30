#pragma once
#include "MsgReceiver.hpp"

class Condition :public MsgReceiver {
private:
	
public:
    virtual bool isConditionMet()=0;

    virtual void receiveMsgData(DataMessage* t_msg)=0;
    
};