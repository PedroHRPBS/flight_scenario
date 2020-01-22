#include "ErrorMsg.hpp"

msg_type ErrorMsg::getType()
{
	return msg_type::ERROR;
}

const int ErrorMsg::getSize()
{
	return sizeof(ErrorMsg);
}