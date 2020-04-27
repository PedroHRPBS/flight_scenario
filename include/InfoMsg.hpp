#pragma once
#include "common_srv/DataMessage.hpp"

class InfoMsg : public DataMessage{

public:
    bool armed;
    int number_of_waypoints;

    msg_type getType();
	const int getSize();
};