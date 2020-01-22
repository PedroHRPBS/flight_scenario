#include "InfoMsg.hpp"

msg_type InfoMsg::getType()
{
	return msg_type::INFO;
}

const int InfoMsg::getSize()
{
	return sizeof(InfoMsg);
}