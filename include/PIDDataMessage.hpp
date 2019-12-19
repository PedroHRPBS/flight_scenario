#pragma once
#include "common_types.hpp"
#include "DataMessage.hpp"
#include "PID_values.hpp"
class PIDDataMessage: public DataMessage {

public:
    
    msg_type getType();
    const int getSize() ;
    PID_parameters PIDdata;
	block_id target_block;
    PIDDataMessage();
    ~PIDDataMessage();
};