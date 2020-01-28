#include "ROSUnit_RestNormSettingsClnt.hpp"

ROSUnit_RestNormSettingsClnt::ROSUnit_RestNormSettingsClnt(ros::NodeHandle& t_main_handler) : ROSUnit(t_main_handler) {
    _clnt_rest_norm_settings = t_main_handler.serviceClient<positioning_system::Restricted_Norm_Settings>("restricted_norm_settings");
}   

ROSUnit_RestNormSettingsClnt::~ROSUnit_RestNormSettingsClnt() {

}

void ROSUnit_RestNormSettingsClnt::receive_msg_data(DataMessage* t_msg){
    
    if(t_msg->getType() == msg_type::RESTNORMREF_SETTINGS){

        RestrictedNormRefSettingsMsg* _settings_msg = (RestrictedNormRefSettingsMsg*)t_msg;
        
        positioning_system::Restricted_Norm_Settings srv;
        srv.request.enabled = _settings_msg->enabled;
        srv.request.delete_existing_waypoints = _settings_msg->delete_existing_waypoints;
        srv.request.max_norm = _settings_msg->getMaxNorm();


        bool success = _clnt_rest_norm_settings.call(srv);

        if (success)
        {
            ROS_INFO("SETTINGS. ENABLED: %d, DELETE_WAYPOINTS: %d, MAX_NORM: %f",srv.request.enabled, srv.request.delete_existing_waypoints,srv.request.max_norm);
        }
        else 
        {
            ROS_ERROR("Failed to call service /switch_block");
        }
    }
    
}