#pragma once
#include "Condition.hpp"
#include "common_types.hpp"
#include "common_srv/IntegerMsg.hpp"

class DNNConfirmationCondition : public Condition {

private:

	bool _isConditionMet=false;
    control_system _cs;

public:
	
    bool isConditionMet();
    void receiveMsgData(DataMessage* t_msg);

    DNNConfirmationCondition(control_system);
    
};
