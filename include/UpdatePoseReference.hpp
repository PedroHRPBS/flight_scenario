#pragma once
#include "FlightElement.hpp"
#include "MessageToBlock.hpp"
#include "UpdatePoseMessage.hpp"
#include "MessageToBlock.hpp"
class UpdatePoseReference : public FlightElement{
private:
	
public:
	block_id target_block;
    UpdatePoseMessage pose_reference;

    void perform();

    void receive_msg_data(DataMessage* t_msg);
    
    UpdatePoseReference();
    ~UpdatePoseReference();
};
