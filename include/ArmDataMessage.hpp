#pragma once
#include "DataMessage.hpp"

class ArmDataMessage: public DataMessage {

public:
    ArmDataMessage();
    ~ArmDataMessage();
    msg_type getType();
    const int getSize();
    int isArmed=0;

};