#include "SwitchBlock.hpp"


void SwitchBlock::perform(){

    this->emit_message((DataMessage*)&switch_msg);
}

void SwitchBlock::receive_msg_data(DataMessage* t_msg){

}

SwitchBlock::SwitchBlock(){

}
SwitchBlock::~SwitchBlock(){

}
