#pragma once

class PID_parameters {

public:

	float kp, ki, kd, kdd, anti_windup, dt; //TODO Calculate dt dynamically
	uint8_t en_pv_derivation;
	   
};