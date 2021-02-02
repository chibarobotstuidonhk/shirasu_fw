/*
 * shell.cpp
 *
 *  Created on: 2020/10/25
 *      Author: ryuni
 */
#include "microshell.h"
#include "msconf.h"
#include "mscmd.h"

#include "MotorCtrl.hpp"
#include "dwt.hpp"
#include "conf.hpp"

#include "shell.hpp"
#include "main.h"
#include "string.h"
#include "stdio.h"

extern MotorCtrl control;

extern "C" {
	void cdc_puts(char *str);
	char cdc_getc();
	void cdc_put(char c);
	uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);
}

namespace{
	uint8_t sendbuf[] = "shell>";
	char buf[MSCONF_MAX_INPUT_LENGTH];

	MICROSHELL ms;
	MSCMD mscmd;

	typedef struct {
		void (*puts)(char *str);
	} USER_OBJECT;

	USER_OBJECT usrobj = {
			.puts = cdc_puts,
	};

	void action_hook(MSCORE_ACTION action)
	{
	}

	MSCMD_USER_RESULT usrcmd_help(MSOPT *msopt, MSCMD_USER_OBJECT usrobj)
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

	MSCMD_USER_RESULT usrcmd_led_toggle(MSOPT *msopt, MSCMD_USER_OBJECT usrobj)
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

	MSCMD_USER_RESULT usrcmd_target(MSOPT *msopt, MSCMD_USER_OBJECT usrobj)
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

	MSCMD_USER_RESULT usrcmd_monitor(MSOPT *msopt, MSCMD_USER_OBJECT usrobj)
	{
		USER_OBJECT *uo = (USER_OBJECT *)usrobj;
		char buf[MSCONF_MAX_INPUT_LENGTH];
		int argc;
		msopt_get_argc(msopt, &argc);
		if(argc == 1){
			char str[256];
			sprintf(str,"%d\r\n",control.conf_diag);
			uo->puts(str);
		}
		else if(argc == 2){
			msopt_get_argv(msopt, 1, buf, sizeof(buf));
			uint8_t monitor;
			sscanf(buf,"%d",&monitor);
			control.conf_diag = static_cast<MotorCtrl::Diagnostic>(monitor);
		}
		else cdc_puts("too many arguments!\r\n");
		return 0;
	}

	MSCMD_USER_RESULT usrcmd_get(MSOPT *msopt, MSCMD_USER_OBJECT usrobj)
	{
		USER_OBJECT *uo = (USER_OBJECT *)usrobj;
		char buf[MSCONF_MAX_INPUT_LENGTH];
		int argc;
		msopt_get_argc(msopt, &argc);
		for (int i = 0; i < argc; i++) {
			char str[256]={};
			msopt_get_argv(msopt, i, buf, sizeof(buf));
			if (strcmp(buf, "CUR") == 0){
				sprintf(str,"current:%f[A]\r\n",control.data.current);
			}
			else if(strcmp(buf,"PROS")==0){
				sprintf(str,"process time:%f[ms]\r\n",dwt::ProcessTim::get_process_time());
			}
			else if(strcmp(buf,"FREQ")==0){
				sprintf(str,"frequency:%f[Hz]\r\n",dwt::Frequency::get_process_frequency());
			}
			else if(strcmp(buf,"VEL")==0){
				sprintf(str,"velocity:%f[rad/s]\r\n",control.data.velocity);
			}
			else if(strcmp(buf,"POS")==0){
				sprintf(str,"position:%d[pulse]\r\n",control.data.position_pulse);
			}
			else if(strcmp(buf,"ID")==0){
				sprintf(str,"id:%3x\r\n",control.can_id);
			}
			uo->puts(str);
		}
		return 0;
	}


	MSCMD_USER_RESULT usrcmd_bid(MSOPT *msopt, MSCMD_USER_OBJECT usrobj)
	{
		USER_OBJECT *uo = (USER_OBJECT *)usrobj;
		char buf[MSCONF_MAX_INPUT_LENGTH];
		int argc;
		msopt_get_argc(msopt, &argc);
		if(argc == 1){
			char str[256]={};
			sprintf(str,"0X%3x\r\n",control.can_id);
			uo->puts(str);
		}
		else if(argc == 2){
			msopt_get_argv(msopt, 1, buf, sizeof(buf));
			uint16_t id;
			sscanf(buf,"%3x",&id);
			if(control.SetBID(id)){
				cdc_puts("invalid BID\r\n");
				cdc_puts("BID should be 0x003 < bid < 0x800  and multiples of 4\r\n");
			}
		}
		else cdc_puts("too many arguments!\r\n");
		return 0;
	}

	MSCMD_USER_RESULT usrcmd_ppr(MSOPT *msopt, MSCMD_USER_OBJECT usrobj)
	{
		USER_OBJECT *uo = (USER_OBJECT *)usrobj;
		char buf[MSCONF_MAX_INPUT_LENGTH];
		int argc;
		msopt_get_argc(msopt, &argc);
		if(argc == 1){
			char str[256]={};
			sprintf(str,"%f[Pulses per Revolution]\r\n",control.GetCPR()/4);
			uo->puts(str);
		}
		else if(argc == 2){
			msopt_get_argv(msopt, 1, buf, sizeof(buf));
			Float_Type ppr;
			sscanf(buf,"%f",&ppr);
			control.SetCPR(ppr*4);
		}
		else cdc_puts("too many arguments!\r\n");
		return 0;
	}

	MSCMD_USER_RESULT usrcmd_cpr(MSOPT *msopt, MSCMD_USER_OBJECT usrobj)
	{
		USER_OBJECT *uo = (USER_OBJECT *)usrobj;
		char buf[MSCONF_MAX_INPUT_LENGTH];
		int argc;
		msopt_get_argc(msopt, &argc);
		if(argc == 1){
			char str[256]={};
			sprintf(str,"%f[Counts per Revolution]\r\n",control.GetCPR());
			uo->puts(str);
		}
		else if(argc == 2){
			msopt_get_argv(msopt, 1, buf, sizeof(buf));
			Float_Type cpr;
			sscanf(buf,"%f",&cpr);
			control.SetCPR(cpr);
		}
		else cdc_puts("too many arguments!\r\n");
		return 0;
	}

	MSCMD_USER_RESULT usrcmd_kpr(MSOPT *msopt, MSCMD_USER_OBJECT usrobj)
	{
		USER_OBJECT *uo = (USER_OBJECT *)usrobj;
		char buf[MSCONF_MAX_INPUT_LENGTH];
		int argc;
		msopt_get_argc(msopt, &argc);
		if(argc == 1){
			char str[256]={};
			sprintf(str,"%f[A/(rad/s)]\r\n",control.GetKp());
			uo->puts(str);
		}
		else if(argc == 2){
			msopt_get_argv(msopt, 1, buf, sizeof(buf));
			Float_Type kpr;
			sscanf(buf,"%f",&kpr);
			control.SetKp(kpr);
		}
		else cdc_puts("too many arguments!\r\n");
		return 0;
	}

	MSCMD_USER_RESULT usrcmd_kit(MSOPT *msopt, MSCMD_USER_OBJECT usrobj)
	{
		USER_OBJECT *uo = (USER_OBJECT *)usrobj;
		char buf[MSCONF_MAX_INPUT_LENGTH];
		int argc;
		msopt_get_argc(msopt, &argc);
		if(argc == 1){
			char str[256]={};
			sprintf(str,"%f[A/(rad/s)]\r\n",control.GetKi());
			uo->puts(str);
		}
		else if(argc == 2){
			msopt_get_argv(msopt, 1, buf, sizeof(buf));
			Float_Type kit;
			sscanf(buf,"%f",&kit);
			control.SetKi(kit);
		}
		else cdc_puts("too many arguments!\r\n");
		return 0;
	}

	MSCMD_USER_RESULT usrcmd_kvp(MSOPT *msopt, MSCMD_USER_OBJECT usrobj)
	{
		USER_OBJECT *uo = (USER_OBJECT *)usrobj;
		char buf[MSCONF_MAX_INPUT_LENGTH];
		int argc;
		msopt_get_argc(msopt, &argc);
		if(argc == 1){
			char str[256]={};
			sprintf(str,"%f[(rad/s) / rad]\r\n",control.GetKv());
			uo->puts(str);
		}
		else if(argc == 2){
			msopt_get_argv(msopt, 1, buf, sizeof(buf));
			Float_Type kvp;
			sscanf(buf,"%f",&kvp);
			control.SetKv(kvp);
		}
		else cdc_puts("too many arguments!\r\n");
		return 0;
	}

	MSCMD_USER_RESULT usrcmd_vsp(MSOPT *msopt, MSCMD_USER_OBJECT usrobj)
	{
		USER_OBJECT *uo = (USER_OBJECT *)usrobj;
		char buf[MSCONF_MAX_INPUT_LENGTH];
		int argc;
		msopt_get_argc(msopt, &argc);
		if(argc == 1){
			char str[256]={};
			sprintf(str,"%f[V]\r\n",control.supply_voltage);
			uo->puts(str);
		}
		else cdc_puts("too many arguments!\r\n");
		return 0;
	}

	MSCMD_USER_RESULT usrcmd_default_mode(MSOPT *msopt, MSCMD_USER_OBJECT usrobj)
	{
		USER_OBJECT *uo = (USER_OBJECT *)usrobj;
		char buf[MSCONF_MAX_INPUT_LENGTH];
		int argc;
		msopt_get_argc(msopt, &argc);
		if(argc == 1){
			switch(control.GetDefaultMode()){
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
				default:
					cdc_puts("disable\r\n");
					break;
			}
		}
		else if(argc == 2){
			msopt_get_argv(msopt, 1, buf, sizeof(buf));
			if(strcmp(buf,"DUT")==0) control.SetDefaultMode(MotorCtrl::Mode::duty);
			else if(strcmp(buf,"CUR")==0) control.SetDefaultMode(MotorCtrl::Mode::current);
			else if(strcmp(buf,"VEL")==0) control.SetDefaultMode(MotorCtrl::Mode::velocity);
			else if(strcmp(buf,"POS")==0) control.SetDefaultMode(MotorCtrl::Mode::position);
			else control.SetDefaultMode(MotorCtrl::Mode::disable);
		}
		else cdc_puts("too many arguments!\r\n");
		return 0;
	}


	MSCMD_USER_RESULT usrcmd_temp(MSOPT *msopt, MSCMD_USER_OBJECT usrobj)
	{
		USER_OBJECT *uo = (USER_OBJECT *)usrobj;
		char buf[MSCONF_MAX_INPUT_LENGTH];
		int argc;
		msopt_get_argc(msopt, &argc);
		if(argc == 1){
			char str[256]={};
			sprintf(str,"%f[degrees]\r\n",control.temperature);
			uo->puts(str);
		}
		else cdc_puts("too many arguments!\r\n");
		return 0;
	}

	MSCMD_USER_RESULT usrcmd_test(MSOPT *msopt, MSCMD_USER_OBJECT usrobj)
	{
		USER_OBJECT *uo = (USER_OBJECT *)usrobj;
		char buf[MSCONF_MAX_INPUT_LENGTH];
		int argc;
		msopt_get_argc(msopt, &argc);
		if(argc == 2){
			msopt_get_argv(msopt, 1, buf, sizeof(buf));
			Float_Type target;
			sscanf(buf,"%f",&target);
			cdc_puts("tick,position_pulse,target_position_pulse,velocity,target_velocity,current,target_current,target_voltage,temperature\r\n");
			control.SetMode(control.GetDefaultMode());
			control.conf_diag = MotorCtrl::Diagnostic::usb;
			HAL_Delay(3000);
			control.SetTarget(target);
			HAL_Delay(3000);
			control.SetTarget(0);
			control.SetMode(MotorCtrl::Mode::disable);
			control.conf_diag = MotorCtrl::Diagnostic::disable;
		}

		return 0;
	}



	MSCMD_USER_RESULT usrcmd_mode(MSOPT *msopt, MSCMD_USER_OBJECT usrobj)
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
			if(strcmp(buf,"DUT")==0) control.SetMode(MotorCtrl::Mode::duty);
			else if(strcmp(buf,"CUR")==0) control.SetMode(MotorCtrl::Mode::current);
			else if(strcmp(buf,"VEL")==0) control.SetMode(MotorCtrl::Mode::velocity);
			else if(strcmp(buf,"POS")==0) control.SetMode(MotorCtrl::Mode::position);
			else if(strcmp(buf,"DEF")==0) control.SetMode(control.GetDefaultMode());
			else control.SetMode(MotorCtrl::Mode::disable);
		}
		else cdc_puts("too many arguments!\r\n");
		return 0;
	}

	MSCMD_USER_RESULT usrcmd_flash(MSOPT *msopt, MSCMD_USER_OBJECT usrobj)
	{
		control.WriteConfig();
		writeConf();
		return 0;
	}

	MSCMD_COMMAND_TABLE table[] = {
		{	"MODE"		,	usrcmd_mode },
		{   "BID"    ,   usrcmd_bid		},
		{   "PPR"    ,   usrcmd_ppr	},
		{   "CPR"    ,   usrcmd_cpr	},
		{   "KPR"    ,   usrcmd_kpr	},
		{   "KIT"    ,   usrcmd_kit	},
		{	"VSP"	,	 usrcmd_vsp},
		{	"TEMP"	,	usrcmd_temp},
		{   "KVP"    ,   usrcmd_kvp	},
		{   "DEF"    ,   usrcmd_default_mode },
		{	"MONITOR", usrcmd_monitor	},
		{   "TEST"    ,   usrcmd_test	},
		{   "WCFG"    ,   usrcmd_flash	},
		{   "HELP",     usrcmd_help     },
		{   "?",        usrcmd_help     },
		{   "t_led",  usrcmd_led_toggle	},
		{ 	"TARGET" ,   usrcmd_target	},
		{   "GET"    ,   usrcmd_get		},
	};

}

namespace shell{

	void init(){
		microshell_init(&ms, cdc_put, cdc_getc, action_hook);
		mscmd_init(&mscmd, table, sizeof(table) / sizeof(table[0]), &usrobj);
		cdc_puts("\r\n");
	}

	void update(){
		MSCMD_USER_RESULT r;
		CDC_Transmit_FS(sendbuf,sizeof(sendbuf));
		microshell_getline(&ms, buf, sizeof(buf));
		mscmd_execute(&mscmd, buf, &r);
	}

}


