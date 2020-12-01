/*
 * MotorCtrl.cpp
 *
 *  Created on: Sep 10, 2020
 *      Author: ryu
 */

#include <MotorCtrl.hpp>
#include "dwt.hpp"
#include "stdio.h"
#include <cmath>
#include "conf.hpp"
#include "main.h"

extern "C"{
	uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);
}

void MotorCtrl::Init(TIM_HandleTypeDef* tim_pwm,ADC_HandleTypeDef* adc_master,ADC_HandleTypeDef* adc_slave){
	this->tim_pwm = tim_pwm;
	this->adc_master = adc_master;
	this->adc_slave = adc_slave;
	ccr_max = __HAL_TIM_GET_AUTORELOAD(tim_pwm);

	ReadConfig();

	motor.R = 0.289256;
	motor.L = 0.000144628;
	current_controller.Kp = 800*motor.L;
	Float_Type pole = motor.R / motor.L;
	current_controller.Ki = pole * current_controller.Kp;

	HAL_ADCEx_Calibration_Start(adc_master, ADC_SINGLE_ENDED);
	HAL_ADCEx_Calibration_Start(adc_slave, ADC_SINGLE_ENDED);
	HAL_ADC_Start(adc_slave);
	HAL_ADCEx_MultiModeStart_DMA(adc_master, &adc_buff[0].ADCDualConvertedValue, MotorCtrl::ADC_DATA_SIZE*2);

	SetMode(Mode::disable);
}

void MotorCtrl::Start(){
	current_controller.reset();
	velocity_controller.reset();
	position_controller.reset();
	motor.reset();

	__HAL_TIM_SET_COMPARE(tim_pwm, TIM_CHANNEL_1, 1);
	__HAL_TIM_SET_COMPARE(tim_pwm, TIM_CHANNEL_2, 1);
	HAL_TIM_PWM_Start(tim_pwm, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(tim_pwm, TIM_CHANNEL_2);
}

void MotorCtrl::Stop(){
	HAL_TIM_PWM_Stop(tim_pwm, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(tim_pwm, TIM_CHANNEL_2);
}

// @param d duty ratio multiplied by 1000, where 1000 represents 100% duty ratio.
void MotorCtrl::SetDuty(int d){

    if (d < -1000 || 1000 < d)
    {
        return;
    }

    else if (0 < d)
    {
    	__HAL_TIM_SET_COMPARE(tim_pwm, TIM_CHANNEL_1, 0);
    	__HAL_TIM_SET_COMPARE(tim_pwm, TIM_CHANNEL_2, d*ccr_max/1000);
    }
    else if (d < 0)
    {
    	__HAL_TIM_SET_COMPARE(tim_pwm, TIM_CHANNEL_1, -d*ccr_max/1000);
    	__HAL_TIM_SET_COMPARE(tim_pwm, TIM_CHANNEL_2, 0);
    }

    else
    {
    	__HAL_TIM_SET_COMPARE(tim_pwm, TIM_CHANNEL_1, 0);
    	__HAL_TIM_SET_COMPARE(tim_pwm, TIM_CHANNEL_2, 0);
    }
}

void MotorCtrl::SetVoltage(Float_Type v){
	SetDuty(v/supply_voltage*1000);
	data.voltage=v;
}

void MotorCtrl::SetMode(Mode Mode){
	switch(Mode){
		case Mode::duty:
			Control = &MotorCtrl::ControlDuty;
			target_voltage = 0;
			break;
		case Mode::current:
			Control = &MotorCtrl::ControlCurrent;
			target_current = 0;
			break;
		case Mode::velocity:
			Control = &MotorCtrl::ControlVelocity;
			target_velocity = 0;
			break;
		case Mode::position:
			Control = &MotorCtrl::ControlPosition;
			target_position_pulse = data.position_pulse;
			break;
		default:
			Control = &MotorCtrl::ControlDisable;
	}
	if(Mode != Mode::disable){
		Start();
		HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED_YELLOW_GPIO_Port, LED_YELLOW_Pin, GPIO_PIN_SET);
	}
	else{
		Stop();
		HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LED_YELLOW_GPIO_Port, LED_YELLOW_Pin, GPIO_PIN_RESET);
	}
	mode = Mode;
	//TODO:ES信号を見る
}

MotorCtrl::Mode MotorCtrl::GetMode(void)const{
	return mode;
}

void MotorCtrl::SetTarget(Float_Type target){
	switch(mode){
		case Mode::duty:
			target_voltage = target;
			break;
		case Mode::current:
			target_current = target;
			break;
		case Mode::velocity:
			target_velocity = target;
			break;
		default:
			;
	}
}

Float_Type MotorCtrl::GetTarget()const{
	switch(mode){
		case Mode::duty:
			return target_voltage;
		case Mode::current:
			return target_current;
		case Mode::velocity:
			return target_velocity;
		default:
			;
	}
}

uint8_t MotorCtrl::SetCPR(Float_Type cpr)
{
	if(std::isfinite(cpr)){
		Kh = 2 * M_PI / (cpr * T);
		return 0;
	}
	else return -1;
}

Float_Type MotorCtrl::GetCPR(void)
{
    return 2 * M_PI / (Kh * T);
}

// set proportional gain Kp
uint8_t MotorCtrl::SetKp(Float_Type kp)
{
    if (kp < 0 || !std::isfinite(kp)){
    	velocity_controller.Kp = 0;
    	return -1;
    }

    velocity_controller.Kp = kp;
    return 0;
}

Float_Type MotorCtrl::GetKp(void)
{
    return velocity_controller.Kp;
}

// integral gain
uint8_t MotorCtrl::SetKi(Float_Type ki)
{
    if (ki < 0 || !std::isfinite(ki)){
    	velocity_controller.Ki = 0;
    	return -1;
    }

    velocity_controller.Ki = ki;
    return 0;
}

Float_Type MotorCtrl::GetKi(void)
{
    return velocity_controller.Ki;
}

void MotorCtrl::ControlDuty(){
	Float_Type amp = std::abs(data.current);
//	bool sign = std::signbit(data.current);
	if(amp > current_lim_pusled){
		SetMode(Mode::disable);
	}
	else SetVoltage(target_voltage);
}

void MotorCtrl::ControlCurrent(){
	//current limit
	Float_Type amp = std::abs(target_current);
	bool sign = std::signbit(target_current);
	if(amp > current_lim_pusled) amp = current_lim_pusled;
	target_current = sign?-amp:amp;

	SetVoltage(current_controller.update(target_current-data.current));
}

void MotorCtrl::ControlVelocity(){
	target_current = velocity_controller.update(target_velocity-data.velocity);
	ControlCurrent();
}
void MotorCtrl::ControlPosition(){}

void MotorCtrl::invoke(uint16_t* buf){
	dwt::Frequency freq;
	dwt::ProcessTim tim;

	//current
	int32_t sum=0;
	for(int i=0;i<ADC_DATA_SIZE*2;i+=2){
		sum+=buf[i]-buf[i+1];
	}
	data.current = sum*3.3/4096/20/ADC_DATA_SIZE*1000;

//	static uint16_t i;
//	static uint32_t sum_i;
//	static constexpr uint16_t SAMPLE_SIZE=2000;
//	static Float_Type sample[SAMPLE_SIZE];
//
//	//velocity,position
    int16_t pulse = static_cast<int16_t>(TIM2->CNT);
    TIM2->CNT = 0;
	data.velocity = pulse * Kh;
    data.position_pulse += pulse;
//
//    sum_i -= sample[i];
//    sample[i]=data.current*data.current;
//    sum_i += sample[i];
//	if(i<SAMPLE_SIZE-1) i++;
//	else i=0;
//

	(this->*Control)();
}

void MotorCtrl::ReadConfig()
{
	readConf();
	can_id = confStruct.can_id;
	SetCPR(confStruct.cpr);
	SetKp(confStruct.Kp);
	SetKi(confStruct.Ki);
}

void MotorCtrl::WriteConfig()
{
    confStruct.can_id = can_id;
    confStruct.cpr = GetCPR();
    confStruct.Kp = GetKp();
    confStruct.Ki = GetKi();
}
