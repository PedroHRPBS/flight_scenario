#pragma once
#include "DataMessage.hpp"

class UpdatePoseMessage_FS : public DataMessage{

private:
    float _x, _y, _z, _yaw;
    int _mask;
    msg_type _type;

public:

    float getX();
    float getY();
    float getZ();
    float getYaw();
    int getMask();
    msg_type getType();
    const int getSize();
    void setPoseMessage(float, float, float, float, int);
    
    UpdatePoseMessage_FS();
    ~UpdatePoseMessage_FS();
};