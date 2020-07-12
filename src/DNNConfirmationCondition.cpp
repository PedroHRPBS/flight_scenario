#include "DNNConfirmationCondition.hpp"

DNNConfirmationCondition::DNNConfirmationCondition(control_system t_cs){
    _cs = t_cs;
}

bool DNNConfirmationCondition::isConditionMet(){
	return _isConditionMet;
}

void DNNConfirmationCondition::receiveMsgData(DataMessage* t_msg){
	IntegerMsg* int_msg = (IntegerMsg*)t_msg;
    int cs_data = int_msg->data;

    if(!_isConditionMet){
        _isConditionMet = (cs_data == (int)_cs);
    }
}
    
