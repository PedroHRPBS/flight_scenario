// Example program
#include <iostream>
#include <string>

#pragma once
#include "MsgEmitter.hpp"
#include "MsgReceiver.hpp"

class FlightElement :public msg_emitter, public msg_receiver{

public:

    virtual void perform() = 0;
    virtual void receive_msg_data(DataMessage* t_msg) = 0;
    
    FlightElement();
    ~FlightElement();
};

#pragma once
#include "FlightElement.hpp"

class Arm : public FlightElement{

public:

    void perform(){
        ArmDataMessage _arm_message;
        _arm_message.isArmed=1;
        this->emit_message((DataMessage*)&_arm_message);
    }
    void receive_msg_data(DataMessage* t_msg);
    
    Arm();
    ~Arm();
};


class Disarm : public FlightElement{

public:

    void perform(){
        ArmDataMessage _arm_message;
        _arm_message.isArmed=0;
        this->emit_message((DataMessage*)&_arm_message);
    }
    void receive_msg_data(DataMessage* t_msg);
    
    Arm();
    ~Arm();
};


class ArmDataMessage: public DataMessage {

public:
    
    msg_type getType(){
        return msg_type::arm_update;
    }
    const int getSize() {
        return sizeof(*this);
    }
    int isArmed=0;
    DataMessage();
    ~DataMessage();
};

class PID_parameters {

public:

	float kp, ki, kd, kdd, anti_windup, dt; //TODO Calculate dt dynamically
	uint8_t en_pv_derivation;
	   
};

public enum BlockID {PID_roll};


class PIDDataMessage: public DataMessage {

public:
    
    msg_type getType(){
        return msg_type::pid_data_update;
    }
    const int getSize() {
        return sizeof(*this);
    }
    PID_parameters PIDdata;
	BlockID target_block;
    DataMessage();
    ~DataMessage();
};

class UpdatePIDcontroller : public FlightElement{
private:
	
public:
	PID_parameters PIDdata;
	BlockID target_block;
    void perform(){
        PIDDataMessage _pid_parameters_message;
        _pid_parameters_message.PIDdata=PIDdata;
        this->emit_message((DataMessage*)&_pid_parameters_message);
    }
	
    void receive_msg_data(DataMessage* t_msg);
    
    UpdatePIDcontroller();
    ~UpdatePIDcontroller();
};

class MessageToBlock: public DataMessage {

public:
    
    msg_type getType(){
        return msg_type::ControlMessage;
    }
    const int getSize() {
        return sizeof(*this);
    }
	BlockID target_block;
    DataMessage();
    ~DataMessage();
};

class ResetControllerMessage: public MessageToBlock {

public:
    
    msg_type getType(){
        return msg_type::RestControllerMessage;
    }
    const int getSize() {
        return sizeof(*this);
    }
    ResetControllerMessage();
    ~ResetControllerMessage();
};


class ResetController : public FlightElement{
private:
	
public:
	BlockID target_block;
    void perform(){
        RestControllerMessage _reset_controller_msg;
		((ControlMessage*)_reset_controller_msg)->target_block=this->target_block;
        this->emit_message((DataMessage*)&_reset_controller_msg);
    }

    void receive_msg_data(DataMessage* t_msg);
    
    ResetController();
    ~ResetController();
};


class UpdateReference : public FlightElement{
private:
	
public:
	BlockID target_block;
    void perform(){
        ReferenceMessage _reference_update_msg;
		((ControlMessage*)_reset_controller_msg)->target_block=this->target_block;
        this->emit_message((DataMessage*)&_reset_controller_msg);
    }

    void receive_msg_data(DataMessage* t_msg);
    
    ResetController();
    ~ResetController();
};


class SwitchToBlockMessage: public MessageToBlock {

public:
    
    msg_type getType(){
        return msg_type::SwitchToBlock;
    }
    const int getSize() {
        return sizeof(*this);
    }
    SwitchToBlockMessage();
    ~SwitchToBlockMessage();
};

class SwitchToBlock : public FlightElement{
private:
	
public:
	BlockID target_block;
    void perform(){
        SwitchToBlockMessage _switch_controller_msg;
		((SwitchToBlockMessage*)_switch_controller_msg)->target_block=this->target_block;
        this->emit_message((DataMessage*)&_switch_controller_msg);
    }

    void receive_msg_data(DataMessage* t_msg);
    
    ResetController();
    ~ResetController();
};

class Wait : public FlightElement{
private:
	
public:
	int wait_time_ms;
    void perform(){
        Timer _wait_timer;
		_wait_timer.tick();
		while (_wait_timer.tockMilliSeconds()<wait_time_ms)
		{}
		
    }

    void receive_msg_data(DataMessage* t_msg);
    
    ResetController();
    ~ResetController();
};

class WaitForCondition : public FlightElement{
private:
	
public:
	Condition _wait_condition;
    void perform(){

		while (!_wait_condition.isConditionMet())
		{}
		
    }
    void receive_msg_data(DataMessage* t_msg);
};

class Condition :public msg_receiver {
private:
	
public:
    virtual bool isConditionMet()=0;

    virtual void receive_msg_data(DataMessage* t_msg)=0;
    
};


class SimplePlaneCondition :public Condition {
private:
	bool _isConditionMet=false;
public:
`	Dimension3D selected_dim=Dimension3D::Z;
	double condition_value=1;
	bool condition_met_for_larger=1;
	
    bool isConditionMet(){
		return _isConditionMet;
	}

    void receive_msg_data(DataMessage* t_msg){
		double _condition_var_val=0;
		
		if (selected_dim==Dimension3D::X){
			_condition_var_val=((PositionMsg*) t_msg)->x;
		}
		else if (selected_dim==Dimension3D::Y){
			_condition_var_val=((PositionMsg*) t_msg)->y;
		}	
		else if(selected_dim==Dimension3D::Z){
			_condition_var_val=((PositionMsg*) t_msg)->z;
		}
		if (condition_met_for_larger){
			_isConditionMet=_condition_var_val>condition_value;
		}
		else{
			_isConditionMet=_condition_var_val<condition_value;
		}
	}
    
};
