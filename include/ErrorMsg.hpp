#pragma once
#include "DataMessage.hpp"

class ErrorMsg : public DataMessage {

public:
    bool error = false;

    msg_type getType();
	const int getSize();

};