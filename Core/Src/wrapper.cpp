/*
 * main.cpp
 *
 *  Created on: Aug 11, 2020
 *      Author: ryu
 */
#include <MotorCtrl.hpp>
#include "main.h"
#include "string.h"
#include "stdio.h"
#include "microshell.h"
#include "msconf.h"
#include "mscmd.h"

#include "dwt.hpp"

MotorCtrl control;

extern "C" {
	void cdc_puts(char *str);
	char cdc_getc();
	void cdc_put(char c);
	uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);
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
	}
};



typedef struct {
    void (*puts)(char *str);
} USER_OBJECT;

static void action_hook(MSCORE_ACTION action)
{
}

static MSCMD_USER_RESULT usrcmd_help(MSOPT *msopt, MSCMD_USER_OBJECT usrobj)
{
    USER_OBJECT *uo = (USER_OBJECT *)usrobj;
    uo->puts(
            "system : system command\r\n"
            "config : config command\r\n"
            "help   : help command\r\n"
    		"t_led   : toggle led\r\n"
            );
    return 0;
}

static MSCMD_USER_RESULT usrcmd_led_toggle(MSOPT *msopt, MSCMD_USER_OBJECT usrobj)
{
	USER_OBJECT *uo = (USER_OBJECT *)usrobj;
    char buf[MSCONF_MAX_INPUT_LENGTH];
    int argc;
    msopt_get_argc(msopt, &argc);
    for (int i = 0; i < argc; i++) {
        msopt_get_argv(msopt, i, buf, sizeof(buf));
        if (strcmp(buf, "RED") == 0) HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);
        else if (strcmp(buf, "GREEN") == 0) HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
        else if (strcmp(buf, "YELLOW") == 0) HAL_GPIO_TogglePin(LED_YELLOW_GPIO_Port, LED_YELLOW_Pin);
        else if (strcmp(buf, "CAN") == 0) HAL_GPIO_TogglePin(LED_CAN_GPIO_Port, LED_CAN_Pin);
    }
    return 0;
}

static MSCMD_USER_RESULT usrcmd_target(MSOPT *msopt, MSCMD_USER_OBJECT usrobj)
{
	USER_OBJECT *uo = (USER_OBJECT *)usrobj;
    char buf[MSCONF_MAX_INPUT_LENGTH];
    int argc;
    msopt_get_argc(msopt, &argc);
    if(argc == 1){
    	char str[256];
    	sprintf(str,"target:%lf\r\n",(double)control.GetTarget());
    	uo->puts(str);
    }
    else if(argc == 2){
        msopt_get_argv(msopt, 1, buf, sizeof(buf));
        double target;
        sscanf(buf,"%lf",&target);
        control.SetTarget(target);
    }
    else cdc_puts("too many arguments!\r\n");
    return 0;
}

static MSCMD_USER_RESULT usrcmd_monitor(MSOPT *msopt, MSCMD_USER_OBJECT usrobj)
{
	USER_OBJECT *uo = (USER_OBJECT *)usrobj;
    char buf[MSCONF_MAX_INPUT_LENGTH];
    int argc;
    msopt_get_argc(msopt, &argc);
    control.monitor = true;
    if(argc == 3){
    	msopt_get_argv(msopt, 2, buf, sizeof(buf));
    	uint32_t ms;
    	sscanf(buf,"%d",&ms);
    	HAL_Delay(ms);
    }
    if(argc >= 2){
        msopt_get_argv(msopt, 1, buf, sizeof(buf));
        double target;
        sscanf(buf,"%lf",&target);
        control.SetTarget(target);
    }
    while(cdc_getc()!='\r');
    control.monitor = false;
    return 0;
}

static MSCMD_USER_RESULT usrcmd_get(MSOPT *msopt, MSCMD_USER_OBJECT usrobj)
{
	USER_OBJECT *uo = (USER_OBJECT *)usrobj;
    char buf[MSCONF_MAX_INPUT_LENGTH];
    int argc;
    msopt_get_argc(msopt, &argc);
    for (int i = 0; i < argc; i++) {
    	char str[256]={};
        msopt_get_argv(msopt, i, buf, sizeof(buf));
        if (strcmp(buf, "current") == 0){
        	sprintf(str,"current:%f[A]\r\n",control.data.current);
        }
        else if(strcmp(buf,"process")==0){
        	sprintf(str,"process time:%f[ms]\r\n",dwt::ProcessTim::get_process_time());
        }
        else if(strcmp(buf,"frequency")==0){
        	sprintf(str,"frequency:%f[Hz]\r\n",dwt::Frequency::get_process_frequency());
        }
        else if(strcmp(buf,"velocity")==0){
        	sprintf(str,"velocity:%f[rad/s]\r\n",control.data.velocity);
        }
        else if(strcmp(buf,"position")==0){
        	sprintf(str,"position:%d[pulse]\r\n",control.data.position_pulse);
        }
        uo->puts(str);
    }
    return 0;
}

static MSCMD_USER_RESULT usrcmd_mode(MSOPT *msopt, MSCMD_USER_OBJECT usrobj)
{
	USER_OBJECT *uo = (USER_OBJECT *)usrobj;
    char buf[MSCONF_MAX_INPUT_LENGTH];
    int argc;
    msopt_get_argc(msopt, &argc);
    if(argc == 1){
    	switch(control.GetMode()){
    		case MotorCtrl::Mode::duty:
    			cdc_puts("duty control\r\n");
    			break;
    		case MotorCtrl::Mode::current:
    			cdc_puts("current control\r\n");
    			break;
    		case MotorCtrl::Mode::velocity:
    			cdc_puts("velocity control\r\n");
    			break;
    		case MotorCtrl::Mode::position:
    			cdc_puts("position control\r\n");
    			break;
    		case MotorCtrl::Mode::disable:
    			cdc_puts("disable\r\n");
    			break;
    	}
    }
    else if(argc == 2){
    	msopt_get_argv(msopt, 1, buf, sizeof(buf));
    	if(strcmp(buf,"duty")==0) control.SetMode(MotorCtrl::Mode::duty);
    	else if(strcmp(buf,"current")==0) control.SetMode(MotorCtrl::Mode::current);
    	else if(strcmp(buf,"velocity")==0) control.SetMode(MotorCtrl::Mode::velocity);
    	else if(strcmp(buf,"position")==0) control.SetMode(MotorCtrl::Mode::position);
    	else control.SetMode(MotorCtrl::Mode::disable);

    }
    else cdc_puts("too many arguments!\r\n");
    return 0;
}

static MSCMD_COMMAND_TABLE table[] = {
	{	"mode"		,	usrcmd_mode },
    {   "help",     usrcmd_help     },
    {   "?",        usrcmd_help     },
	{   "t_led",  usrcmd_led_toggle	},
	{ 	"target" ,   usrcmd_target	},
	{	"monitor", usrcmd_monitor	},
	{   "get"    ,   usrcmd_get		},
};

void main_cpp(void)
{
	dwt::init();

	char buf[MSCONF_MAX_INPUT_LENGTH];
	MICROSHELL ms;
	MSCMD mscmd;
	USER_OBJECT usrobj = {
			.puts = cdc_puts,
	};

	microshell_init(&ms, cdc_put, cdc_getc, action_hook);
	mscmd_init(&mscmd, table, sizeof(table) / sizeof(table[0]), &usrobj);

	HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
	HAL_GPIO_WritePin(USB_PULLUP_GPIO_Port, USB_PULLUP_Pin,GPIO_PIN_SET);

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
	control.Init(&htim15,&hadc1,&hadc2);
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL);

	uint8_t sendbuf[] = "shell>";
	while(1){
		MSCMD_USER_RESULT r;
		CDC_Transmit_FS(sendbuf,sizeof(sendbuf));
		microshell_getline(&ms, buf, sizeof(buf));
		mscmd_execute(&mscmd, buf, &r);
	}

}






