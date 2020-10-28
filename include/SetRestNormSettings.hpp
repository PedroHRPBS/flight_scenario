#pragma once
#include "FlightElement.hpp"
#include "MessageToBlock.hpp"
#include "RestrictedNormRefSettingsMsg.hpp"
#include "MessageToBlock.hpp"
#include "PositionMsg.hpp"
#include "common_srv/Vector3DMessage.hpp"

class SetRestNormSettings : public FlightElement {

private:
    Port* _output_port_0;
	bool _enabled, _delete;
    float _max_norm;

public:
    enum ports_id {OP_0};
    RestrictedNormRefSettingsMsg _settings_msg;

    void perform();
    void process(DataMessage* t_msg, Port* t_port) {};
    
    SetRestNormSettings(bool, bool, float);
    ~SetRestNormSettings();
};