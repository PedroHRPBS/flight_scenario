
#include "MessageToBlock.hpp"

    msg_type MessageToBlock::getType(){
        return msg_type::MessageToBlock;
    }
    const int MessageToBlock::getSize() {
        return sizeof(*this);
    }
    MessageToBlock::MessageToBlock(){

    }
    MessageToBlock::~MessageToBlock(){

    }
