/*
 * MotorCtrl.h
 *
 *  Created on: Sep 10, 2020
 *      Author: ryu
 */

#ifndef SRC_MOTORCTRL_H_
#define SRC_MOTORCTRL_H_

#include "stm32f3xx_hal.h"
#include <cmath>

using Float_Type = float;

struct DataStruct{
	Float_Type current;
	Float_Type velocity;
	int32_t position_pulse;
};

class MotorCtrl {
public:
	enum class Mode {duty,current,velocity,position,disable};
	enum class Diagnostic {disable,usb,can};
	void ReadConfig();
	void WriteConfig();
	uint32_t can_id;
private:
	class PI{
	public:
		Float_Type Kp;
		Float_Type Ki;
		Float_Type T;
		void reset()
		{
			e_prev = 0;
		}
		Float_Type update(Float_Type e)
		{
			Float_Type du = Kp*(e - e_prev)+Ki*T*e;
			e_prev = e;
			return du;
		}
	private:
		Float_Type e_prev;

	};

	class Motor{
	public:
		Float_Type R;
		Float_Type L;
//		Float_Type inverse(Float_Type i)
//		{
//			Float_Type y = (L/T+R)*i - L/T*i_prev;
//			i_prev = i;
//			return y;
//		}
//	private:
//		Float_Type i_prev=0;
	};

private:
	TIM_HandleTypeDef* tim_pwm;
	TIM_HandleTypeDef* tim_it;
	ADC_HandleTypeDef* adc_current;
	ADC_HandleTypeDef* adc_sensor;
	uint16_t ccr_arr;
	uint16_t ccr_max;
	Mode mode=Mode::disable;
	Mode default_mode;
	Float_Type target=0;
	Float_Type voltage=0;

	PI current_controller;
	PI velocity_controller;
	PI position_controller;
	Motor motor;

	void SetDuty(int d);
	void Start();
	void Stop();
public:
	DataStruct data;
	Float_Type supply_voltage=20;
	Float_Type temperature=25;
	Float_Type target_duty;
	Float_Type target_voltage;
	Float_Type target_current;
	Float_Type target_velocity;
	int32_t target_position_pulse;
	void Init(TIM_HandleTypeDef*,ADC_HandleTypeDef*,ADC_HandleTypeDef*);
	Mode GetMode()const;
	void SetMode(Mode);
	Float_Type GetTarget()const;
	void SetSupplyVoltage(Float_Type sv);
	void SetTarget(Float_Type);
	void SetVoltage();
	void invoke(uint16_t* buf);
	Float_Type Kh = 2 * M_PI / (2000 / 1e3); // エンコーダ入力[pulse/ctrl]を[rad/s]に変換する係数．kg / Tc．
	Float_Type voltage_lim;
	static constexpr Float_Type current_lim=60;
	static constexpr Float_Type Tc=0.001;
	Diagnostic conf_diag=Diagnostic::disable;
	void Print(void);

	void ControlDuty();
	void ControlCurrent();
	void ControlVelocity();
	void ControlPosition();
	int8_t SetVSP(Float_Type vsp);
	Float_Type GetVSP(void);
	int8_t SetTEMP(Float_Type temp);
	Float_Type GetTEMP(void);
	int8_t SetCPR(Float_Type cpr);
	Float_Type GetCPR(void);
	int8_t SetKp(Float_Type kp);
	Float_Type GetKp(void);
	int8_t SetKi(Float_Type ki);
	Float_Type GetKi(void);
    int8_t SetKv(Float_Type kv);
    Float_Type GetKv(void);
    int8_t SetDefaultMode(Mode dm);
    Mode GetDefaultMode();
    int8_t SetBID(uint32_t bid);
};


#endif /* SRC_MOTORCTRL_H_ */
