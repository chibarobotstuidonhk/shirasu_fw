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
	class PI{
	public:
		void reset(Float_Type Kp,Float_Type Ki)
		{
			this->Kp = Kp;
			this->Ki = Ki;
			u = 0;
			e_prev = 0;
		}
		Float_Type update(Float_Type e)
		{
			u += Kp*(e - e_prev)+Ki*T*e;
			e_prev = e;
			return u;
		}
	private:
		Float_Type u;
		Float_Type e_prev;
		Float_Type Kp;
		Float_Type Ki;
	};

	class Motor{
	public:
		Float_Type R;
		Float_Type L;
		void reset(Float_Type R,Float_Type L)
		{
			this->R = R;
			this->L = L;
			i_prev = 0;
		}
		Float_Type inverse(Float_Type i)
		{
			Float_Type y = (L/T+R)*i - L/T*i_prev;
			i_prev = i;
			return y;
		}
	private:
		Float_Type i_prev;
	};

private:
	TIM_HandleTypeDef* tim_pwm;
	TIM_HandleTypeDef* tim_it;
	uint16_t ccr_max;
	Mode mode=Mode::disable;
	Float_Type target=0;
	Float_Type voltage=0;
	static constexpr Float_Type T=0.00022756559374454696;	//TODO:set automatically
	Float_Type supply_voltage=20;	//TODO:measure supply voltage
	PI current_controller;
	Motor motor;

	void SetDuty(int d);

	void ControlDisable(){};
	void ControlDuty(){};
	void ControlCurrent();
public:
	Float_Type current;
	void Init(TIM_HandleTypeDef* tim_pwm,TIM_HandleTypeDef* tim_it);
	Mode GetMode()const;
	void SetMode(Mode);
	void SetTarget(Float_Type);
	void SetVoltage(Float_Type);
	void (MotorCtrl::*Control)(void)=&MotorCtrl::ControlDisable;
	void invoke(uint16_t* data);
	static constexpr uint16_t ADC_DATA_SIZE=256;
	bool monitor = false;
};

#endif /* SRC_MOTORCTRL_H_ */
