#include "FlightCommandMsg.hpp"

FlightCommandMsg::FlightCommandMsg(){

}
FlightCommandMsg::~FlightCommandMsg(){
    
}

msg_type FlightCommandMsg::getType(){
    return _type;
}
const int FlightCommandMsg::getSize() {
    return sizeof(*this);
}


int FlightCommandMsg::getData(){
    return _data;
}

void FlightCommandMsg::setFlightCommand(int t_data){
    _type = msg_type::FLIGHTCOMMAND;
    _data = t_data;
}
