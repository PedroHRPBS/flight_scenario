#pragma once
#include "ROSUnit.hpp"
#include "RestrictedNormRefSettingsMsg.hpp"
#include <flight_controller/Restricted_Norm_Settings.h>

class ROSUnit_RestNormSettingsClnt :  public ROSUnit{

    private:
        ros::ServiceClient _clnt_rest_norm_settings;
        
    public:
        void receive_msg_data(DataMessage* t_msg);  
        ROSUnit_RestNormSettingsClnt(ros::NodeHandle&);
        ~ROSUnit_RestNormSettingsClnt();
};