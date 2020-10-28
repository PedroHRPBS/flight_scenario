#pragma once
#include "common_srv/ROSUnit.hpp"
#include "RestrictedNormRefSettingsMsg.hpp"
#include <flight_controller/Restricted_Norm_Settings.h>

class ROSUnit_RestNormSettingsClnt :  public ROSUnit{

    private:
        Port* _input_port_0;
        ros::ServiceClient _clnt_rest_norm_settings;
        
    public:
        enum ports_id {IP_0};
        void process(DataMessage* t_msg, Port* t_port);

        ROSUnit_RestNormSettingsClnt(ros::NodeHandle&);
        ~ROSUnit_RestNormSettingsClnt();
};