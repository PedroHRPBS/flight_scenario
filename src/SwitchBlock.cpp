#include "SwitchBlock.hpp"


void SwitchBlock::perform(){

    this->emitMsgUnicastDefault((DataMessage*)&switch_msg);
}

void SwitchBlock::receiveMsgData(DataMessage* t_msg){

}

SwitchBlock::SwitchBlock(){

}
SwitchBlock::~SwitchBlock(){

}
