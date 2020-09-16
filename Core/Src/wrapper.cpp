/*
 * main.cpp
 *
 *  Created on: Aug 11, 2020
 *      Author: ryu
 */
#include "main.h"
#include "string.h"
#include "stdio.h"
#include "microshell.h"
#include "msconf.h"
#include "mscmd.h"

#include "MotorCtrl.h"

MotorCtrl control;
static constexpr uint8_t ADC_CONVERTED_DATA_BUFFER_SIZE = 58;
static uint16_t   ADC1_ConvertedData[ADC_CONVERTED_DATA_BUFFER_SIZE];
static uint16_t   ADC2_ConvertedData[ADC_CONVERTED_DATA_BUFFER_SIZE];
enum class Monitor {left,right,off};
static Monitor monitor=Monitor::off;

extern "C" {
	void cdc_puts(char *str);
	char cdc_getc();
	void cdc_put(char c);
	uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);
	TIM_HandleTypeDef htim15;
	TIM_HandleTypeDef htim3;

	ADC_HandleTypeDef hadc1;
	ADC_HandleTypeDef hadc2;

	void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
	{
		if(hadc->Instance == ADC1)
		{
//			HAL_GPIO_TogglePin(LED_YELLOW_GPIO_Port, LED_YELLOW_Pin);
			uint32_t sum=0;
			for(const uint8_t& data:ADC1_ConvertedData){
				sum+=data;
			}
			control.current_left = sum* 3.3/4096 * 20/ADC_CONVERTED_DATA_BUFFER_SIZE;
			if(monitor==Monitor::left)
			{
				CDC_Transmit_FS((uint8_t*)ADC1_ConvertedData,ADC_CONVERTED_DATA_BUFFER_SIZE*sizeof(uint16_t));
			}
		}
		if(hadc->Instance == ADC2)
		{
//			HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);
			uint32_t sum=0;
			for(const uint8_t& data:ADC2_ConvertedData){
				sum+=data;
			}
			control.current_right = sum* 3.3/4096 * 20/ADC_CONVERTED_DATA_BUFFER_SIZE;
			if(monitor==Monitor::right)
			{
				CDC_Transmit_FS((uint8_t*)ADC2_ConvertedData,ADC_CONVERTED_DATA_BUFFER_SIZE*sizeof(uint16_t));
			}
		}
	}
    /**
     * @brief
     * This is the workhorse of the md201x.
     * this handler is called @ 1 kHz.
     */
	void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
	{
	  if(htim->Instance == TIM3)
	  {
		  control.invoke();
	  }
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
//    	switch(control.GetMode()){
//    		case Mode::duty:
//    			cdc_puts("duty control\r\n");
//    			break;
//    		case Mode::current:
//    			cdc_puts("current control\r\n");
//    			break;
//    		case Mode::velocity:
//    			cdc_puts("velocity control\r\n");
//    			break;
//    		case Mode::position:
//    			cdc_puts("position control\r\n");
//    			break;
//    		case Mode::disable:
//    			cdc_puts("disable\r\n");
//    			break;
//    	}
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
    if(argc >= 2){
        msopt_get_argv(msopt, 1, buf, sizeof(buf));
        if(strcmp(buf,"left")==0) monitor=Monitor::left;
        else if(strcmp(buf,"right")==0) monitor=Monitor::right;
    }
    if(argc ==4){
    	msopt_get_argv(msopt, 3, buf, sizeof(buf));
    	uint32_t ms;
    	sscanf(buf,"%d",&ms);
    	HAL_Delay(ms);
    }
    if(argc >= 3){
        msopt_get_argv(msopt, 2, buf, sizeof(buf));
        double target;
        sscanf(buf,"%lf",&target);
        control.SetTarget(target);
    }
    while(cdc_getc()!='\n');
    monitor=Monitor::off;
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
    		case Mode::duty:
    			cdc_puts("duty control\r\n");
    			break;
    		case Mode::current:
    			cdc_puts("current control\r\n");
    			break;
    		case Mode::velocity:
    			cdc_puts("velocity control\r\n");
    			break;
    		case Mode::position:
    			cdc_puts("position control\r\n");
    			break;
    		case Mode::disable:
    			cdc_puts("disable\r\n");
    			break;
    	}
    }
    else if(argc == 2){
    	msopt_get_argv(msopt, 1, buf, sizeof(buf));
    	if(strcmp(buf,"duty")==0) control.SetMode(Mode::duty);
    	else if(strcmp(buf,"current")==0) control.SetMode(Mode::current);
    	else if(strcmp(buf,"velocity")==0) control.SetMode(Mode::velocity);
    	else if(strcmp(buf,"position")==0) control.SetMode(Mode::position);
    	else control.SetMode(Mode::disable);

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
	{	"monitor", usrcmd_monitor	}
};

void main_cpp(void)
{
	HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
	HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
	HAL_ADC_Start_DMA(&hadc1,(uint32_t *)ADC1_ConvertedData,ADC_CONVERTED_DATA_BUFFER_SIZE);
	HAL_ADC_Start_DMA(&hadc2,(uint32_t *)ADC2_ConvertedData,ADC_CONVERTED_DATA_BUFFER_SIZE);

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
	control.Init(&htim15);
	HAL_TIM_Base_Start_IT(&htim3);

	uint8_t sendbuf[] = "shell>";
	while(1){
		MSCMD_USER_RESULT r;
		CDC_Transmit_FS(sendbuf,sizeof(sendbuf));
		microshell_getline(&ms, buf, sizeof(buf));
		mscmd_execute(&mscmd, buf, &r);
	}

}






