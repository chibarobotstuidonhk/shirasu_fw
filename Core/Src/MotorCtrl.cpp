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
	current_controller.Kp = 1200*motor.L;
	Float_Type pole = motor.R / motor.L;
	current_controller.Ki = pole * current_controller.Kp;

	velocity_controller.Kp = 0.5;
	velocity_controller.Ki = 20;
}

void MotorCtrl::Start(){
	current_controller.reset();
	velocity_controller.reset();
	position_controller.reset();
	motor.reset();

	HAL_ADCEx_Calibration_Start(adc_master, ADC_SINGLE_ENDED);
	HAL_ADCEx_Calibration_Start(adc_slave, ADC_SINGLE_ENDED);
	HAL_ADC_Start(adc_slave);
	HAL_ADCEx_MultiModeStart_DMA(adc_master, &adc_buff[0].ADCDualConvertedValue, MotorCtrl::ADC_DATA_SIZE*2);

	__HAL_TIM_SET_COMPARE(tim_pwm, TIM_CHANNEL_1, 1);
	__HAL_TIM_SET_COMPARE(tim_pwm, TIM_CHANNEL_2, 1);
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
    	__HAL_TIM_SET_COMPARE(tim_pwm, TIM_CHANNEL_2, 0);
    }
    else if (d < 0)
    {
    	__HAL_TIM_SET_COMPARE(tim_pwm, TIM_CHANNEL_1, 0);
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
			target = 0;
			break;
		case Mode::current:
			Control = &MotorCtrl::ControlCurrent;
			target = 0;
			break;
		case Mode::velocity:
			target = 0;
			break;
		case Mode::position:
			target = data.position_pulse;
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
			SetDuty(target*1000);
			break;
		case Mode::current:
			target_current = target;
			break;
		default:
			;
	}
}

Float_Type MotorCtrl::GetTarget()const{
	switch(mode){
		case Mode::duty:
			return 0;
		case Mode::current:
			return target_current;
		default:
			;
	}
}

void MotorCtrl::ControlCurrent(){
	SetVoltage(current_controller.update(target_current-data.current));
}

void MotorCtrl::ControlVelocity(){
	target_current = velocity_controller.update(target_velocity-data.velocity);
}
void MotorCtrl::ControlPosition(){}

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
	}
}

void MotorCtrl::update(){
	static uint16_t i;
	static uint32_t sum;
	static constexpr uint16_t SAMPLE_SIZE=2000;
	static Float_Type sample[SAMPLE_SIZE];

    int16_t pulse = static_cast<int16_t>(TIM2->CNT);
    TIM2->CNT = 0;
	data.velocity = pulse * Kh;
    data.position_pulse += pulse;

    sum -= sample[i];
    sample[i]=data.current*data.current;
    sum += sample[i];
	if(i<SAMPLE_SIZE-1) i++;
	else i=0;

	switch (mode) {
		case MotorCtrl::Mode::position:
			ControlPosition();
		case MotorCtrl::Mode::velocity:
			ControlVelocity();
	}

	//current limit
	Float_Type amp = std::abs(target_current);
	bool sign = std::signbit(target_current);
	if(amp > current_lim_pusled) amp = current_lim_pusled;
	if(sum > current_lim_continuous*current_lim_continuous*SAMPLE_SIZE && amp>current_lim_continuous) amp = 0;
	target_current = sign?-amp:amp;
}

void MotorCtrl::ReadConfig()
{
	readConf();
	this->can_id = confStruct.can_id_cmd;
}

void MotorCtrl::WriteConfig()
{
    confStruct.can_id_cmd = this->can_id;
}
