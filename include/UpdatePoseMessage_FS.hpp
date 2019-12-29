#pragma once
#include "DataMessage.hpp"

class UpdatePoseMessage_FS : public DataMessage{

private:
    float _x, _y, _z, _yaw;
    msg_type _type;
    msg_type_reference _ref_type;

public:

    float getX();
    float getY();
    float getZ();
    float getYaw();
    msg_type getType();
    msg_type_reference getRefType();
    const int getSize();
    void setPoseMessage(float, float, float, float);
    void setPoseX(float);
    void setPoseY(float);
    void setPoseZ(float);
    void setPoseYaw(float);
    
    UpdatePoseMessage_FS();
    ~UpdatePoseMessage_FS();
};