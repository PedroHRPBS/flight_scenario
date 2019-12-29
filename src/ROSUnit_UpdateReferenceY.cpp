#include "ROSUnit_UpdateReferenceY.hpp"

ROSUnit_UpdateReferenceY::ROSUnit_UpdateReferenceY(ros::NodeHandle& t_main_handler) : ROSUnit(t_main_handler){

    _setpoint_position_client = t_main_handler.serviceClient<positioning_system::Update_Y_Reference>("update_referece/y");
}

ROSUnit_UpdateReferenceY::~ROSUnit_UpdateReferenceY() {

}

void ROSUnit_UpdateReferenceY::receive_msg_data(DataMessage* t_msg){
    
    if(t_msg->getType() == msg_type::USERREFERENCE){

        UpdatePoseMessage_FS* ref_msg = (UpdatePoseMessage_FS*)t_msg;
        if(ref_msg->getRefType() == msg_type_reference::Y){
           
            positioning_system::Update_Y_Reference srv;
            srv.request.setpoint_y = ref_msg->getY();
        
            bool success = _setpoint_position_client.call(srv);

            if (success)
            {
                ROS_INFO("NEW Y REFERENCE PUBLISHED: %f", srv.request.setpoint_y);
            }
            else 
            {
                ROS_ERROR("Failed to call service /update_reference/y");
            }
        }
    }
    

}
