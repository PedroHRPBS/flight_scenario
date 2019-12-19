#include "WaitForCondition.hpp"


void WaitForCondition::perform(){

while (!_wait_condition.isConditionMet())
{}

}
void WaitForCondition::receive_msg_data(DataMessage* t_msg){}
