#pragma once
#include "common_types.hpp"
#include "MessageToBlock.hpp"

class ResetControllerMessage: public MessageToBlock {

public:
    
    msg_type getType();
    const int getSize();

    ResetControllerMessage();
    ~ResetControllerMessage();
};