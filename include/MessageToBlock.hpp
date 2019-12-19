#pragma once
#include "common_types.hpp"
#include "DataMessage.hpp"
class MessageToBlock: public DataMessage {

public:
    
    msg_type getType();
    const int getSize();
	block_id target_block;
    MessageToBlock();
    ~MessageToBlock();
};