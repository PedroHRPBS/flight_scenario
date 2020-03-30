#include "SetRestNormSettings.hpp"

SetRestNormSettings::SetRestNormSettings(bool t_enabled, bool t_delete, float t_max_norm){
    _enabled = t_enabled;
    _delete = t_delete;
    _max_norm = t_max_norm;

}

SetRestNormSettings::~SetRestNormSettings(){}

void SetRestNormSettings::perform(){
    _settings_msg.enabled = _enabled;
    _settings_msg.delete_existing_waypoints = _delete;
    _settings_msg.setMaxNorm(_max_norm);
    this->emitMsgUnicastDefault((DataMessage*)&_settings_msg);
}

void SetRestNormSettings::receiveMsgData(DataMessage* t_msg){

}



