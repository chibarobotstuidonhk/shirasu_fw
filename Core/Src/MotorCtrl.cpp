/*
 * MotorCtrl.cpp
 *
 *  Created on: Sep 10, 2020
 *      Author: ryu
 */

#include <MotorCtrl.hpp>
#include "dwt.hpp"
#include "stdio.h"

extern "C"{
	uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);
}

void MotorCtrl::Init(TIM_HandleTypeDef* tim_pwm,ADC_HandleTypeDef* adc_master,ADC_HandleTypeDef* adc_slave){
	this->tim_pwm = tim_pwm;
	this->adc_master = adc_master;
	this->adc_slave = adc_slave;
	ccr_max = __HAL_TIM_GET_AUTORELOAD(tim_pwm);

	motor.R = 0.289256;
	motor.L = 0.0000144628;
	current_controller.Kp = 1000*motor.L;
	Float_Type pole = motor.R / motor.L;
	current_controller.Ki = pole * current_controller.Kp;
}

void MotorCtrl::Start(){
	target = 0;
	current_controller.reset();
	motor.reset();
	HAL_ADCEx_Calibration_Start(adc_master, ADC_SINGLE_ENDED);
	HAL_ADCEx_Calibration_Start(adc_slave, ADC_SINGLE_ENDED);
	HAL_ADC_Start(adc_slave);
	HAL_ADCEx_MultiModeStart_DMA(adc_master, &adc_buff[0].ADCDualConvertedValue, MotorCtrl::ADC_DATA_SIZE*2);

	HAL_TIM_PWM_Start(tim_pwm, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(tim_pwm, TIM_CHANNEL_2);
}

void MotorCtrl::Stop(){
	HAL_TIM_PWM_Stop(tim_pwm, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(tim_pwm, TIM_CHANNEL_2);
	HAL_ADCEx_MultiModeStop_DMA(adc_master);
	HAL_ADC_Stop(adc_slave);
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
			Control = &MotorCtrl::ControlDuty;
			break;
		case Mode::current:
			Control = &MotorCtrl::ControlCurrent;
			break;
		default:
			Control = &MotorCtrl::ControlDisable;
	}
	if(Mode != Mode::disable) Start();
	else Stop();
	mode = Mode;
	//TODO:ES信号を見る
}

MotorCtrl::Mode MotorCtrl::GetMode(void)const{
	return mode;
}

void MotorCtrl::SetTarget(Float_Type target){
	this->target = target;
	switch(mode){
		case Mode::duty:
			SetDuty(target*1000);
			break;
		case Mode::current:
			break;
		default:
			;
	}
}

Float_Type MotorCtrl::GetTarget()const{
	return target;
}

void MotorCtrl::ControlCurrent(){
	SetVoltage(current_controller.update(target-data.current));
}

void MotorCtrl::invoke(uint16_t* buf){
	dwt::Frequency freq;
	dwt::ProcessTim tim;
	int32_t sum=0;
	for(int i=0;i<ADC_DATA_SIZE*2;i+=2){
		sum+=buf[i]-buf[i+1];
	}
	data.current = sum*3.3/4096/20/ADC_DATA_SIZE*1000;
	(this->*Control)();
	if(monitor){
		CDC_Transmit_FS((uint8_t*)buf,ADC_DATA_SIZE*sizeof(uint16_t)*2);
//		CDC_Transmit_FS((uint8_t*)&data,sizeof(data));
	}
}
