/*
 * MotorCtrl.cpp
 *
 *  Created on: Sep 10, 2020
 *      Author: ryu
 */

#include "MotorCtrl.h"

void MotorCtrl::Init(TIM_HandleTypeDef* htim){
	SetMode(Mode::disable);
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
	//TO DO:ES信号を見る
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
	static Float_Type error[2];
	static Float_Type u;
	if(target>0){
		error[1] = error[0];
		error[0] = target - current_left;
		u += 0.1*(error[0]-error[1]) + 0.001*0.05*error[0];
		SetDuty(u*1000);
	}
	else if(target<0){

	}
}
