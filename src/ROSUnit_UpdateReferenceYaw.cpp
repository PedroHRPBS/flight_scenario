#include "ROSUnit_UpdateReferenceYaw_FS.hpp"

ROSUnit_UpdateReferenceYaw_FS::ROSUnit_UpdateReferenceYaw_FS(ros::NodeHandle& t_main_handler) : ROSUnit(t_main_handler){

    _setpoint_position_client = t_main_handler.serviceClient<positioning_system::Update_Yaw_Reference>("update_reference/yaw");
}

ROSUnit_UpdateReferenceYaw_FS::~ROSUnit_UpdateReferenceYaw_FS() {

}

void ROSUnit_UpdateReferenceYaw_FS::receive_msg_data(DataMessage* t_msg){
    
    if(t_msg->getType() == msg_type::USERREFERENCE){

        UpdatePoseMessage_FS* ref_msg = (UpdatePoseMessage_FS*)t_msg;
        if(ref_msg->getRefType() == msg_type_reference::YAW){
            
            positioning_system::Update_Yaw_Reference srv;
            srv.request.setpoint_yaw = ref_msg->getYaw();
        
            bool success = _setpoint_position_client.call(srv);

            if (success)
            {
                ROS_INFO("NEW Yaw REFERENCE PUBLISHED: %f", srv.request.setpoint_yaw);
            }
            else 
            {
                ROS_ERROR("Failed to call service /update_reference/yaw");
                _error_msg.error = true;
                this->emit_message((DataMessage*) &_error_msg);
            }
        }
    }
    

}
