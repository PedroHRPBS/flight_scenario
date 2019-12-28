#pragma once
#include "DataMessage.hpp"

class SwitchBlockMsg : public DataMessage {

private:
    msg_type _type;
    block_id _block_in, _block_out;

public:

    msg_type getType();
    const int getSize();
    void setSwitchBlockMsg(block_id t_block_out, block_id t_block_in);
    block_id getBlockToSwitchIn();
    block_id getBlockToSwitchOut();
    SwitchBlockMsg();
    ~SwitchBlockMsg();
};