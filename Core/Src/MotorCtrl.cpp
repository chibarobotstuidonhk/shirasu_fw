/*
 * MotorCtrl.cpp
 *
 *  Created on: Sep 10, 2020
 *      Author: ryu
 */

#include "MotorCtrl.h"

void MotorCtrl::Init(TIM_HandleTypeDef* htim){
	m_tim = htim;
	ccr_max = __HAL_TIM_GET_AUTORELOAD(m_tim);
	SetDuty(0);
	HAL_TIM_PWM_Start(m_tim, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(m_tim, TIM_CHANNEL_2);
}

// @param d duty ratio multiplied by 1000, where 1000 represents 100% duty ratio.
void MotorCtrl::SetDuty(int d){

    if (d < -1000 || 1000 < d)
    {
        return;
    }

    if (0 < d)
    {
//        TIM1->CCR1 = d * ccr_max / 1000;
//        TIM1->CCR2 = 0;
    	__HAL_TIM_SET_COMPARE(m_tim, TIM_CHANNEL_1, d*ccr_max/1000);
    	__HAL_TIM_SET_COMPARE(m_tim, TIM_CHANNEL_2, 1);
    }
    else if (d < 0)
    {
//        TIM1->CCR1 = 0;
//        TIM1->CCR2 = -d * ccr_max / 1000;
    	__HAL_TIM_SET_COMPARE(m_tim, TIM_CHANNEL_1, 1);
    	__HAL_TIM_SET_COMPARE(m_tim, TIM_CHANNEL_2, -d*ccr_max/1000);
    }
    else
    {
//        TIM1->CCR1 = 0;
//        TIM1->CCR2 = 0;
    	__HAL_TIM_SET_COMPARE(m_tim, TIM_CHANNEL_1, 0);
    	__HAL_TIM_SET_COMPARE(m_tim, TIM_CHANNEL_2, 0);
    }
}

void MotorCtrl::Control(void){

}
