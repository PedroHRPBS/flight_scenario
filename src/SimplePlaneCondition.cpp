#include "SimplePlaneCondition.hpp"

bool SimplePlaneCondition::isConditionMet(){
	return _isConditionMet;
}

void SimplePlaneCondition::receiveMsgData(DataMessage* t_msg){
	double _condition_var_val=0;
	
	if (selected_dim==Dimension3D::X){
		_condition_var_val=((PositionMsg*) t_msg)->x;
	}
	else if (selected_dim==Dimension3D::Y){
		_condition_var_val=((PositionMsg*) t_msg)->y;
	}	
	else if(selected_dim==Dimension3D::Z){
		_condition_var_val=((PositionMsg*) t_msg)->z;
	}
	if (condition_met_for_larger){
		_isConditionMet=_condition_var_val>condition_value;
	}
	else{
		_isConditionMet=_condition_var_val<condition_value;
	}
}
    
