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
    /**
     * @brief
     * This is the workhorse of the md201x.
     * this handler is called @ 1 kHz.
     */
	void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
	{
		control.update();
//		can.send(can_test,0x200);
		can.led_process();
	}
	void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
	{
		uint8_t cmd;
		if(can.receive(cmd,control.can_id)){
			switch(cmd){
			case 0:
				break;
			}
		}
		can.endit();//割り込み終了
	}
};

void main_cpp(void)
{
	dwt::init();
	can.init(control.can_id);
	shell::init();

	HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
	HAL_GPIO_WritePin(USB_PULLUP_GPIO_Port, USB_PULLUP_Pin,GPIO_PIN_SET);

	control.Init(&htim15,&hadc1,&hadc2);
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL);

	while(1){
		shell::update();
	}

}






