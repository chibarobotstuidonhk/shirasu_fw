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
	TIM_HandleTypeDef htim15;
	TIM_HandleTypeDef htim2;
	TIM_HandleTypeDef htim3;

	ADC_HandleTypeDef hadc1;
	ADC_HandleTypeDef hadc2;

	void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
	{
		control.invoke(control.adc_buff[MotorCtrl::ADC_DATA_SIZE].ADCConvertedValue);
	}
	void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
	{
		control.invoke(control.adc_buff[0].ADCConvertedValue);
	}

	//2Hz
	void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
	{
		HAL_ADCEx_InjectedStart(&hadc2);
		HAL_ADCEx_InjectedStart(&hadc1);

		if(HAL_GPIO_ReadPin(EMS_GPIO_Port, EMS_Pin) == GPIO_PIN_RESET)
		{
			control.SetMode(MotorCtrl::Mode::disable);
		}

		if(HAL_ADCEx_InjectedPollForConversion(&hadc1,100)==HAL_OK){
			uint32_t adc_voltage = HAL_ADCEx_InjectedGetValue(&hadc1,1);
			control.supply_voltage = adc_voltage*3.3*11/4096;
			uint32_t adc_temp = HAL_ADCEx_InjectedGetValue(&hadc2,1);
			double B = 3434;
			double R_25 = 10e3;
			double R_inf = R_25*exp(-B/(25+173.15));
			double R_load = 3.3e3;
			double R_measure = (4096*R_load)/(double)adc_temp - R_load;
			control.temperature = B/log(R_measure/R_inf) - 173.15;
			if(control.temperature > 40) control.SetMode(MotorCtrl::Mode::disable);
			if(control.monitor) can.send(control.temperature,0x7fb);
		}
	}

	void update(){
		static constexpr uint32_t stream_interval = 1;
		static uint32_t last_stream_time = 0;
		if (HAL_GetTick() - last_stream_time > stream_interval){
			if(control.monitor){
				can.send(control.data.Vemf/0.0080087, 0x7fa);
//				can.send(control.data.voltage,0x7fc);
				can.send(control.data.current,0x7fd);
				can.send(control.data.velocity,0x7fe);
				can.led_process();
				last_stream_time = HAL_GetTick();
			}
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
				break;//TODO:事前に設定したモード
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
	dwt::init();
	shell::init();

	HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);

	control.Init(&htim15,&hadc1,&hadc2);
	can.init(control.can_id);
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL);

	while(1){
		shell::update();
	}

}






