#pragma once
#include <stdint.h>

enum class uav_control_states {WARMING_UP=0, LANDED=1, ERROR=2, UTILITY=3, TAKING_OFF=4, 
                               LANDING=5, FOLLOWING_TRAJECTORY=6, HOVERING=7};