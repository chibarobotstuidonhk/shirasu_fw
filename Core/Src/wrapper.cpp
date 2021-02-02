/*
 * main.cpp
 *
 *  Created on: Aug 11, 2020
 *      Author: ryu
 */
#include "MotorCtrl.hpp"
#include "main.h"


#include "dwt.hpp"
#include "CanClass.hpp"
#include "shell.hpp"

MotorCtrl control;
CanClass can;

extern "C" {
	TIM_HandleTypeDef htim1;
	TIM_HandleTypeDef htim2;
	TIM_HandleTypeDef htim3;

	ADC_HandleTypeDef hadc1;
	ADC_HandleTypeDef hadc2;

	void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc){
		int32_t data = HAL_ADCEx_InjectedGetValue(hadc,2) - HAL_ADCEx_InjectedGetValue(hadc,1);
		control.data.current = data*3.3/4096/50*1000;
		switch(control.GetMode()){
			case MotorCtrl::Mode::current:
			case MotorCtrl::Mode::velocity:
			case MotorCtrl::Mode::position:
				control.ControlCurrent();
				break;
			case MotorCtrl::Mode::duty:
				control.ControlDuty();
				break;
		}
	}

	void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
	{

	  //30Hz
	  if(htim->Instance == TIM3)
	  {
			can.led_process();
			if(HAL_GPIO_ReadPin(EMS_GPIO_Port, EMS_Pin) == GPIO_PIN_RESET)
			{
				control.SetMode(MotorCtrl::Mode::disable);
			}
			if(HAL_ADCEx_InjectedPollForConversion(&hadc1,0)==HAL_OK){
				uint32_t adc_voltage = HAL_ADCEx_InjectedGetValue(&hadc1,1);
				control.SetSupplyVoltage(adc_voltage*3.3*11/4096);
				uint32_t adc_temp = HAL_ADCEx_InjectedGetValue(&hadc1,2);
				double B = 3434;
				double R_25 = 10e3;
				double R_inf = R_25*exp(-B/(25+273.15));
				double R_load = 3.3e3;
				double R_measure = (4096*R_load)/(double)adc_temp - R_load;
				control.temperature = B/log(R_measure/R_inf) - 273.15;
				if(control.temperature > 75) control.SetMode(MotorCtrl::Mode::disable);
				if(control.conf_diag == MotorCtrl::Diagnostic::can) can.send(control.temperature,0x7fb);
				HAL_ADCEx_InjectedStart(&hadc1);
			}
	  }
	}

//	void HAL_TIMEx_BreakCallback(TIM_HandleTypeDef *htim){
//		control.SetMode(MotorCtrl::Mode::disable);
//	}
//	void HAL_TIMEx_Break2Callback(TIM_HandleTypeDef *htim){
//		control.SetMode(MotorCtrl::Mode::disable);
//	}

//	void HAL_COMP_TriggerCallback(COMP_HandleTypeDef *hcomp){
//		HAL_GPIO_TogglePin(LED_CAN_GPIO_Port, LED_CAN_Pin);
//	}
	//1kHz
	void update(){
		switch(control.conf_diag){
			case MotorCtrl::Diagnostic::usb:
				control.Print();
				break;
			case MotorCtrl::Diagnostic::can:
				//can.send(control.data.voltage,0x7fc);
				can.send(control.data.current,0x7fd);
				//can.send(control.data.velocity,0x7fe);
				break;
		}

		//velocity,position
		int16_t pulse = static_cast<int16_t>(TIM2->CNT);
		TIM2->CNT = 0;
		control.data.velocity = pulse * control.Kh;
		control.data.position_pulse += pulse;
		switch(control.GetMode()){
			case MotorCtrl::Mode::position:
				control.ControlPosition();
			case MotorCtrl::Mode::velocity:
				control.ControlVelocity();
				break;
		}
	}

	void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
	{
		uint8_t cmd;
		Float_Type data;
		if(can.receive(data,control.can_id+1)){
			control.SetTarget(data);
		}
		else if(can.receive(cmd,control.can_id)){
			switch(cmd){
			case 0:
				control.SetMode(MotorCtrl::Mode::disable);
				break;
			case 1:
				control.SetMode(control.GetDefaultMode());
				break;
			case 2:
				control.SetMode(MotorCtrl::Mode::current);
				break;
			case 3:
				control.SetMode(MotorCtrl::Mode::velocity);
				break;
			case 4:
				control.SetMode(MotorCtrl::Mode::position);
				break;
			}
		}

		can.endit();//割り込み終了
	}
};

void main_cpp(void)
{
	constexpr uint32_t delay = 100;
	for(uint8_t i=0;i<3;i++){
		HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
		HAL_Delay(delay);
		HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED_YELLOW_GPIO_Port, LED_YELLOW_Pin, GPIO_PIN_SET);
		HAL_Delay(delay);
		HAL_GPIO_WritePin(LED_YELLOW_GPIO_Port, LED_YELLOW_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
		HAL_Delay(delay);
	}
	HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);

	shell::init();

	control.Init(&htim1,&hadc2,&hadc1);
	can.init(control.can_id);

	HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL);
	HAL_TIM_Base_Start_IT(&htim3);

	while(1){
		shell::update();
	}

}






