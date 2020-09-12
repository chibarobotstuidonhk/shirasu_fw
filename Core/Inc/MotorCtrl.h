/*
 * MotorCtrl.h
 *
 *  Created on: Sep 10, 2020
 *      Author: ryu
 */

#ifndef SRC_MOTORCTRL_H_
#define SRC_MOTORCTRL_H_

#include "stm32f3xx_hal.h"

class MotorCtrl {
	using Float_Type = float;
public:
	void Init(TIM_HandleTypeDef* htim);
	void SetDuty(int d);
	void Control(void);
private:
	TIM_HandleTypeDef* m_tim;
	uint16_t ccr_max;
};

#endif /* SRC_MOTORCTRL_H_ */
