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

namespace md{

using Float_Type = float;

struct DataStruct{
	Float_Type current;
	Float_Type velocity;
	int32_t position_pulse;
};

enum class Mode {
	disable,
	duty,
	current,
	velocity,
	position,
	homing
};
enum class Diagnostic {
	disable,
	usb,
	can
};
enum class Error {
	none,
	out_of_operating_voltage,
	out_of_operating_temperature
};
enum class Cmd {
	shutdown,
	recover,
	home,
	get_status,
	recover_current,
	recover_velocity,
	recover_position
};

class PI{
public:
	Float_Type Kp=0;
	Float_Type Ki=0;
	Float_Type T=1/1e3;
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


class MotorCtrl {
public:
	void ReadConfig();
	void WriteConfig();

	void Init(TIM_HandleTypeDef*,ADC_HandleTypeDef*,ADC_HandleTypeDef*);
	void UpdatePulse(int16_t pulse);
	void UpdateCurrent(int32_t data);

	Diagnostic conf_diag=Diagnostic::disable;
	void Print();

	void ControlDuty();
	void ControlCurrent();
	void ControlVelocity();
	void ControlPosition();

	 void ResetPosition(Float_Type offset = 0.0);

	//setter
	int8_t SetTEMP(Float_Type temp);
	int8_t SetCPR(Float_Type cpr);
	int8_t SetKp(Float_Type kp);
	int8_t SetKi(Float_Type ki);
	int8_t SetKv(Float_Type kv);
	int8_t SetDefaultMode(Mode dm);
	int8_t SetBID(uint32_t bid);
	void SetMode(Mode);
	void SetTarget(Float_Type);
	int8_t SetVSP(Float_Type sv);
	void SetVoltage();
	int8_t SetHVL(Float_Type hvl);

	//getter
	Float_Type GetTEMP() const;
	Float_Type GetCPR() const;
	Float_Type GetKp() const;
	Float_Type GetKi() const;
    Float_Type GetKv() const;
    Mode GetDefaultMode() const;
    uint32_t GetBID() const;
    Mode GetMode() const;
    Float_Type GetTarget()const;
    Float_Type GetVSP() const;
    //TODO:GetVoltage
    Error GetError() const;
    Float_Type GetHVL() const;
    Float_Type GetPOS() const;
private:
    //Handle
	TIM_HandleTypeDef* tim_pwm;
	TIM_HandleTypeDef* tim_it;
	ADC_HandleTypeDef* adc_current;
	ADC_HandleTypeDef* adc_sensor;

	//Limit
	Float_Type voltage_lim;
	static constexpr Float_Type current_lim=40;
	static constexpr Float_Type temperature_lim[2] = {0,75};
	static constexpr Float_Type vsp_lim[2] = {11.4,30};

	//Controller
	PI current_controller;
	PI velocity_controller;
	PI position_controller;

	//Mode
	Mode mode=Mode::disable;
	Mode default_mode;

	//Current and Target value
	DataStruct current;
	DataStruct target;
	Float_Type target_duty;
	Float_Type target_voltage;
	Float_Type supply_voltage;
	Float_Type temperature;

	//Parameters
	static constexpr Float_Type Tc=0.001;
	Float_Type Kh=2 * M_PI / (2000 * Tc);
	Float_Type HomingVelocity=0;
	uint16_t ccr_arr;
	uint16_t ccr_max;
	Motor motor;
	uint32_t can_id=0x7fc;

	Error error=Error::none;

	void SetDuty(int d);
	void Start();
	void Stop();
};

}

#endif /* SRC_MOTORCTRL_H_ */
