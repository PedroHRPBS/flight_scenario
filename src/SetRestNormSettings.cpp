#include "SetRestNormSettings.hpp"

SetRestNormSettings::SetRestNormSettings(bool t_enabled, bool t_delete, float t_max_norm){
    _output_port_0 = new OutputPort(ports_id::OP_0, this);
    _ports = {_output_port_0};
    _enabled = t_enabled;
    _delete = t_delete;
    _max_norm = t_max_norm;

}

SetRestNormSettings::~SetRestNormSettings(){}

void SetRestNormSettings::perform(){
    _settings_msg.enabled = _enabled;
    _settings_msg.delete_existing_waypoints = _delete;
    _settings_msg.setMaxNorm(_max_norm);
    _output_port_0->receiveMsgData((DataMessage*)&_settings_msg);
}