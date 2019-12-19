#pragma once
#include "common_types.hpp"
#include "DataMessage.hpp"
#include "PID_parameters.hpp"
class PIDDataMessage: public DataMessage {

public:
    
    msg_type getType(){
        return msg_type::pid_data_update;
    }
    const int getSize() {
        return sizeof(*this);
    }
    PID_parameters PIDdata;
	block_id target_block;
    PIDDataMessage();
    ~PIDDataMessage();
};