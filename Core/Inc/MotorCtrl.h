/*
 * MotorCtrl.h
 *
 *  Created on: Sep 10, 2020
 *      Author: ryu
 */

#ifndef SRC_MOTORCTRL_H_
#define SRC_MOTORCTRL_H_

#include "stm32f3xx_hal.h"

enum class Mode {duty,current,velocity,position,disable};

class MotorCtrl {
	using Float_Type = float;
private:
	TIM_HandleTypeDef* m_tim;
	uint16_t ccr_max;
	Mode mode=Mode::disable;
	Float_Type target=0;

	void SetDuty(int d);

	void ControlDisable(){};
	void ControlDuty(){};
	void ControlCurrent();
public:
	Float_Type current_left;
	Float_Type current_right;
	void Init(TIM_HandleTypeDef* htim);
	Mode GetMode()const;
	void SetMode(Mode);
	void SetTarget(Float_Type);
	void (MotorCtrl::*Control)(void)=&MotorCtrl::ControlDisable;
	void invoke(){
		(this->*Control)();
	}
};

#endif /* SRC_MOTORCTRL_H_ */
