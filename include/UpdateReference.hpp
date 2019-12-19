#pragma once
#include "FlightElement.hpp"
#include "MessageToBlock.hpp"
#include "ReferenceMessage.hpp"
#include "MessageToBlock.hpp"
class UpdateReference : public FlightElement{
private:
	
public:
	block_id target_block;
    void perform();

    void receive_msg_data(DataMessage* t_msg);
    
    UpdateReference();
    ~UpdateReference();
};
