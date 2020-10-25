#pragma once
#include "common_types.hpp"
#include "common_srv/common_types.hpp"
#include "common_srv/DataMessage.hpp"
class MessageToBlock: public DataMessage {

public:
    
    msg_type getType();
    const int getSize();
	block_id target_block;
    MessageToBlock();
    ~MessageToBlock();
};