/*
 * MotorCtrl.cpp
 *
 *  Created on: Sep 10, 2020
 *      Author: ryu
 */

#include "MotorCtrl.h"
#include "dwt.hpp"

extern "C"{
	uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);
}

void MotorCtrl::Init(TIM_HandleTypeDef* tim_pwm,TIM_HandleTypeDef* tim_it){
	SetMode(Mode::disable);
	this->tim_pwm = tim_pwm;
	this->tim_it = tim_it;
	ccr_max = __HAL_TIM_GET_AUTORELOAD(tim_pwm);

	motor.reset(0.289256,0.0000144628);
	Float_Type Kp = 1000*motor.L;
	Float_Type pole = motor.R / motor.L;
	Float_Type Ki = pole * Kp;
	current_controller.reset(Kp, Ki);

	SetDuty(0);
	HAL_TIM_PWM_Start(tim_pwm, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(tim_pwm, TIM_CHANNEL_2);
}

// @param d duty ratio multiplied by 1000, where 1000 represents 100% duty ratio.
void MotorCtrl::SetDuty(int d){

    if (d < -1000 || 1000 < d)
    {
        return;
    }

    if (0 < d)
    {
    	__HAL_TIM_SET_COMPARE(tim_pwm, TIM_CHANNEL_1, d*ccr_max/1000);
    	__HAL_TIM_SET_COMPARE(tim_pwm, TIM_CHANNEL_2, 1);
    }
    else if (d < 0)
    {
    	__HAL_TIM_SET_COMPARE(tim_pwm, TIM_CHANNEL_1, 1);
    	__HAL_TIM_SET_COMPARE(tim_pwm, TIM_CHANNEL_2, -d*ccr_max/1000);
    }
    else
    {
    	__HAL_TIM_SET_COMPARE(tim_pwm, TIM_CHANNEL_1, 0);
    	__HAL_TIM_SET_COMPARE(tim_pwm, TIM_CHANNEL_2, 0);
    }
}

void MotorCtrl::SetVoltage(Float_Type v){
	SetDuty(v/supply_voltage*1000);
	voltage=v;
}

void MotorCtrl::SetMode(Mode Mode){
	switch(Mode){
		case Mode::duty:
			MotorCtrl::Control = &MotorCtrl::ControlDuty;
			break;
		case Mode::current:
			MotorCtrl::Control = &MotorCtrl::ControlCurrent;
			break;
		default:
			SetDuty(0);
			MotorCtrl::Control = &MotorCtrl::ControlDisable;
	}
	mode = Mode;
	//TODO:ES信号を見る
}

Mode MotorCtrl::GetMode(void)const{
	return mode;
}

void MotorCtrl::SetTarget(Float_Type target){
	switch(mode){
		case Mode::duty:
			SetDuty(target*1000);
			break;
		case Mode::current:
			this->target = target;
			break;
		default:
			;
	}
}

void MotorCtrl::ControlCurrent(){
	Float_Type Vemf = voltage - motor.inverse(current); //TODO:Filter
//	SetVoltage(current_controller.update(target-current)+Vemf);
	SetVoltage(current_controller.update(target-current));
}

void MotorCtrl::invoke(uint16_t* data){
	dwt::Frequency freq;
	dwt::ProcessTim tim;
	int32_t sum=0;
	for(int i=0;i<ADC_DATA_SIZE*2;i+=2){
		sum+=data[i]-data[i+1];
	}
	current = sum*3.3/4096/20/ADC_DATA_SIZE*1000;
	(this->*Control)();
	if(monitor)	CDC_Transmit_FS((uint8_t*)data,ADC_DATA_SIZE*sizeof(uint16_t)*2);
}
