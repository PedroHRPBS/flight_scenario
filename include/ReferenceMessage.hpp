#pragma once
#include "common_srv/DataMessage.hpp"
#include "common_srv/Vector3D.hpp"

class ReferenceMessage : public DataMessage {

private:
    msg_type _type;
    float _data;

public:

    msg_type getType();
    const int getSize();
    float getData();
    
    ReferenceMessage(Vector3D<float>);
    ReferenceMessage(float);
    ReferenceMessage();

    void setReferenceMessage(Vector3D<float>);
    void setReferenceMessage(float);

    ~ReferenceMessage();
};