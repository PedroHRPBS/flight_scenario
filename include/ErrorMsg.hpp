#pragma once
#include "common_srv/DataMessage.hpp"

class ErrorMsg : public DataMessage {

public:
    bool error = false;

    msg_type getType();
	const int getSize();

};