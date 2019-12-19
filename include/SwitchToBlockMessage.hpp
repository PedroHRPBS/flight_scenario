#pragma once
#include "common_types.hpp"
#include "MessageToBlock.hpp"

class SwitchToBlockMessage: public MessageToBlock {

public:
    
    msg_type getType();
    const int getSize();
    SwitchToBlockMessage();
    ~SwitchToBlockMessage();
};