/*
 * MotorCtrl.cpp
 *
 *  Created on: Sep 10, 2020
 *      Author: ryu
 */

#include <cstdio>
#include <cmath>
#include "conf.hpp"
#include "main.h"
#include "MotorCtrl.hpp"

using namespace md;

extern "C"{
	uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);
}

void MotorCtrl::Init(TIM_HandleTypeDef* tim_pwm,ADC_HandleTypeDef* adc_current,ADC_HandleTypeDef* adc_sensor){
	this->tim_pwm = tim_pwm;
	this->adc_current = adc_current;
	this->adc_sensor = adc_sensor;

	ccr_arr = __HAL_TIM_GET_AUTORELOAD(tim_pwm);
	ccr_max = (__HAL_TIM_GET_AUTORELOAD(tim_pwm)+1)*0.9-1;

	ReadConfig();

	motor.R = 0.289256;
	motor.L = 0.000144628;
	current_controller.Kp = 1500*motor.L;
	Float_Type pole = motor.R / motor.L;
	current_controller.Ki = pole * current_controller.Kp;
	current_controller.T = 1.0/16e3;

	velocity_controller.T = Tc;

	position_controller.Ki = 0; //P制御
	position_controller.T = Tc;

	SetMode(Mode::disable);
}

void MotorCtrl::Start(){
	current_controller.reset();
	velocity_controller.reset();
	position_controller.reset();

	error = Error::none;

	if (HAL_ADCEx_Calibration_Start(adc_current, ADC_SINGLE_ENDED) != HAL_OK){
		Error_Handler();
	}
	if(HAL_ADCEx_InjectedStart_IT(adc_current)!= HAL_OK){
		Error_Handler();
	}

	if (HAL_ADCEx_Calibration_Start(adc_sensor, ADC_SINGLE_ENDED) != HAL_OK){
		Error_Handler();
	}
	if(HAL_ADCEx_InjectedStart(adc_sensor)!= HAL_OK){
		Error_Handler();
	}

	__HAL_TIM_SET_COMPARE(tim_pwm, TIM_CHANNEL_1, 1);
	__HAL_TIM_SET_COMPARE(tim_pwm, TIM_CHANNEL_2, 1);
	__HAL_TIM_SET_COMPARE(tim_pwm, TIM_CHANNEL_3, ccr_max-32);
	HAL_TIM_PWM_Start(tim_pwm, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(tim_pwm, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(tim_pwm, TIM_CHANNEL_3);
}

void MotorCtrl::Stop(){
	HAL_TIM_PWM_Stop(tim_pwm, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(tim_pwm, TIM_CHANNEL_2);
	HAL_TIM_PWM_Stop(tim_pwm, TIM_CHANNEL_3);

	if(HAL_ADCEx_InjectedStop_IT(adc_current)!= HAL_OK){
		Error_Handler();
	}
	if(HAL_ADCEx_InjectedStop(adc_sensor)!= HAL_OK){
		Error_Handler();
	}

}

void MotorCtrl::UpdatePulse(int16_t pulse){
	current.velocity = pulse * Kh;
	current.position_pulse += pulse;
	switch(GetMode()){
		case Mode::position:
			ControlPosition();
		case Mode::velocity:
		case Mode::homing:
			ControlVelocity();
			break;
	}
}

void MotorCtrl::UpdateCurrent(int32_t data){
	this->current.current = data*3.3/4096/50*1000;
	switch(GetMode()){
		case Mode::current:
		case Mode::velocity:
		case Mode::position:
		case Mode::homing:
			ControlCurrent();
			break;
		case Mode::duty:
			ControlDuty();
			break;
	}
}

void MotorCtrl::ResetPosition(Float_Type offset)
{
	if(GetMode()!=Mode::disable) return;
	target.position_pulse = current.position_pulse = offset / (Kh * Tc);
}

// @param d duty ratio multiplied by 1000, where 1000 represents 100% duty ratio.
void MotorCtrl::SetDuty(int d){

    if (d < -900 || 900 < d)
    {
        return;
    }

    else if (0 < d)
    {
    	__HAL_TIM_SET_COMPARE(tim_pwm, TIM_CHANNEL_1, 1);
    	__HAL_TIM_SET_COMPARE(tim_pwm, TIM_CHANNEL_2, d*ccr_arr/1000);
    }
    else if (d < 0)
    {
    	__HAL_TIM_SET_COMPARE(tim_pwm, TIM_CHANNEL_1, -d*ccr_arr/1000);
    	__HAL_TIM_SET_COMPARE(tim_pwm, TIM_CHANNEL_2, 1);
    }

    else
    {
    	__HAL_TIM_SET_COMPARE(tim_pwm, TIM_CHANNEL_1, 0);
    	__HAL_TIM_SET_COMPARE(tim_pwm, TIM_CHANNEL_2, 0);
    }
}

int8_t MotorCtrl::SetVSP(Float_Type sv){
	supply_voltage = sv;
	if(vsp_lim[0] < sv && sv < vsp_lim[1]){
		voltage_lim = supply_voltage * (ccr_max + 1) / (ccr_arr + 1);
		return 0;
	}
	else{
		SetMode(Mode::disable);
		error = Error::out_of_operating_voltage;
		return -1;
	}
}

Float_Type MotorCtrl::GetVSP() const{
	return supply_voltage;
}

void MotorCtrl::SetVoltage(){
	Float_Type amp = std::abs(target_voltage);
	bool sign = std::signbit(target_voltage);
	if(amp > voltage_lim) amp = voltage_lim;
	target_voltage = sign?-amp:amp;
	SetDuty(target_voltage/supply_voltage*1000);
}

void MotorCtrl::SetMode(Mode Mode){
	if(HAL_GPIO_ReadPin(EMS_GPIO_Port, EMS_Pin) == GPIO_PIN_RESET || Mode==Mode::disable)
	{
		Stop();
		mode = Mode::disable;
		return;
	}

	switch(Mode){
		case Mode::duty:
			target_duty = 0;
			break;
		case Mode::current:
			target.current = 0;
			break;
		case Mode::velocity:
			target.velocity = 0;
			break;
		case Mode::position:
			target.position_pulse = current.position_pulse;
			break;
		case Mode::homing:
			if(HAL_GPIO_ReadPin(DIN_A_GPIO_Port, DIN_A_Pin)==GPIO_PIN_RESET){
				ResetPosition();
		        return;
			}
			else target.velocity = HomingVelocity;
			break;
		default:
			;
	}
	Start();
	mode = Mode;
}

Mode MotorCtrl::GetMode(void) const{
	return mode;
}

void MotorCtrl::SetTarget(Float_Type tar){
	switch(mode){
		case Mode::duty:
			target_duty = tar*1000;
			break;
		case Mode::current:
			target.current = tar;
			break;
		case Mode::velocity:
			target.velocity = tar;
			break;
		case Mode::position:
			target.position_pulse = (int)roundf(tar / (Kh * Tc));
			break;
		default:
			;
	}
}

Float_Type MotorCtrl::GetTarget() const{
	switch(mode){
		case Mode::duty:
			return target_voltage;
		case Mode::current:
			return target.current;
		case Mode::velocity:
			return target.velocity;
		case Mode::position:
			return target.position_pulse * Kh * Tc;
		default:
			;
	}
}

int8_t MotorCtrl::SetCPR(Float_Type cpr){
	if(std::isfinite(cpr)){
		Kh = 2 * M_PI / (cpr * Tc);
		return 0;
	}
	else return -1;
}

Float_Type MotorCtrl::GetCPR(void) const{
    return 2 * M_PI / (Kh * Tc);
}

// set proportional gain Kp
int8_t MotorCtrl::SetKp(Float_Type kp)
{
    if (kp < 0 || !std::isfinite(kp)){
    	return -1;
    }

    velocity_controller.Kp = kp;
    return 0;
}

Float_Type MotorCtrl::GetKp(void) const{
    return velocity_controller.Kp;
}

// integral gain
int8_t MotorCtrl::SetKi(Float_Type ki){
    if (ki < 0 || !std::isfinite(ki)){
    	return -1;
    }

    velocity_controller.Ki = ki;
    return 0;
}

Float_Type MotorCtrl::GetKi(void) const{
    return velocity_controller.Ki;
}

int8_t MotorCtrl::SetKv(Float_Type kv){
    // Kv is NOT allowed to be negative value.
    if (kv < 0 || !std::isfinite(kv)){
    	return -1;
    }

    position_controller.Kp = kv;
    return 0;
}

Float_Type MotorCtrl::GetKv(void) const{
    return position_controller.Kp;
}

int8_t MotorCtrl::SetDefaultMode(Mode dm){
	switch(dm){
		case Mode::duty:
		case Mode::current:
		case Mode::velocity:
		case Mode::position:
			default_mode = dm;
			return 0;
		default:
			default_mode = Mode::disable;
			return -1;
	}
}

Mode MotorCtrl::GetDefaultMode() const{
	return default_mode;
}

int8_t MotorCtrl::SetBID(uint32_t bid){
    if ((0x4 <= bid) && (bid <= 0x7ff) && (bid % 4 == 0)){
    	can_id = bid;
    	return 0;
    }
    else{
    	return -1;
    }

}

uint32_t MotorCtrl::GetBID() const{
	return can_id;
}

int8_t MotorCtrl::SetTEMP(Float_Type temp){
	if(temperature_lim[0] < temp && temp < temperature_lim[1]){
		temperature = temp;
		return 0;
	}
	else{
		SetMode(Mode::disable);
		error = Error::out_of_operating_temperature;
		return -1;
	}
}

Float_Type MotorCtrl::GetTEMP() const{
	return temperature;
}

int8_t MotorCtrl::SetHVL(Float_Type hvl){
	if(std::isfinite(hvl)){
		HomingVelocity = hvl;
		return 0;
	}
	else return -1;
}

Float_Type MotorCtrl::GetHVL() const{
	return HomingVelocity;
}

Float_Type MotorCtrl::GetPOS() const{
	return current.position_pulse*Kh*Tc;
}

Error MotorCtrl::GetError() const{
	return error;
}

void MotorCtrl::ControlDuty(){
	Float_Type amp = std::abs(current.current);
	if(amp > current_lim){
		SetMode(Mode::disable);
	}
	else SetDuty(target_duty);
}

void MotorCtrl::ControlCurrent(){
	//current limit
	Float_Type amp = std::abs(target.current);
	bool sign = std::signbit(target.current);
	if(amp > current_lim) amp = current_lim;
	target.current = sign?-amp:amp;
	target_voltage += current_controller.update(target.current-current.current);
	SetVoltage();
}

void MotorCtrl::ControlVelocity(){
	target.current += velocity_controller.update(target.velocity-current.velocity);
}
void MotorCtrl::ControlPosition(){
	target.velocity += position_controller.update((target.position_pulse-current.position_pulse) * Kh * Tc);
}

void MotorCtrl::ReadConfig(){
	readConf();
	SetBID(confStruct.can_id);
	SetHVL(confStruct.HomVel);
	SetDefaultMode(static_cast<Mode>(confStruct.default_mode));
	SetCPR(confStruct.cpr);
	SetKp(confStruct.Kp);
	SetKi(confStruct.Ki);
	SetKv(confStruct.Kv);
}

void MotorCtrl::WriteConfig(){
    confStruct.can_id = GetBID();
    confStruct.HomVel = GetHVL();
    confStruct.default_mode = static_cast<uint8_t>(GetDefaultMode());
    confStruct.cpr = GetCPR();
    confStruct.Kp = GetKp();
    confStruct.Ki = GetKi();
    confStruct.Kv = GetKv();
}

void MotorCtrl::Print(){
    char buf[128];
    int ret;
//    ret = std::sprintf(buf, "%06lu,%+03d,%+03d,%+3.3f,%+3.3f,%+3.3f,%+3.3f,%+3.3f,%+3.3f\r\n", HAL_GetTick(),
//            current.position_pulse, target.position_pulse,
//			(float)current.velocity,(float)target.velocity,
//            (float)current.current,(float)target.current,
//			(float)target_voltage,(float)temperature);
    ret = std::sprintf(buf, "%06lu,%+3.3f,%+3.3f\r\n", HAL_GetTick(),
			(float)current.velocity,(float)target.velocity);

    if (ret < 0)
    {
        return;
    }

    CDC_Transmit_FS((uint8_t*)buf, ret);
}
