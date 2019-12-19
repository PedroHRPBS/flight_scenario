#pragma once
#include "Condtion.hpp"
#include "comm_types.hpp"

class SimplePlaneCondition :public Condtion {
private:
	bool _isConditionMet=false;
public:
`	Dimension3D selected_dim=Dimension3D::Z;
	double condition_value=1;
	bool condition_met_for_larger=1;
	
    bool isConditionMet();

    void receive_msg_data(DataMessage* t_msg);
    
};
