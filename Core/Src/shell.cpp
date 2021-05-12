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
#include "conf.hpp"

#include "shell.hpp"
#include "main.h"
#include <cstring>
#include <cstdio>

using namespace md;

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
	char tx_buf[128];

	MICROSHELL ms;
	MSCMD mscmd;

	void invalid_value(const char * name, double value)
	{
		int ret = std::sprintf(tx_buf, "--> Invalid value for %s: %lf\r\n", name, value);
		if(0 < ret) cdc_puts(tx_buf);
	}

	void invalid_value(const char * name, const char * param)
	{
		int ret = std::sprintf(tx_buf, "--> Invalid value for %s: %s\r\n", name, param);
		if(0 < ret) cdc_puts(tx_buf);
	}

	void valid_value_set(const char * name, const char * unit, double value)
	{
		int ret = std::sprintf(tx_buf, "--> Set %s: %lf [%s]\r\n", name, value, unit);
		if(0 < ret) cdc_puts(tx_buf);
	}

	void valid_value_set(const char * name, const char * param)
	{
		int ret = std::sprintf(tx_buf, "--> Set %s: %s\r\n", name, param);
		if(0 < ret) cdc_puts(tx_buf);
	}

	void dump_value(const char * name, const char * unit, double value)
	{
	    int ret = std::sprintf(tx_buf, "--> Current %s: %lf [%s]\r\n", name, value, unit);
	    if(0 < ret) cdc_puts(tx_buf);
	}

	void dump_value(const char * name, const char * param)
	{
	    int ret = std::sprintf(tx_buf, "--> Current %s: %s\r\n", name, param);
	    if(0 < ret) cdc_puts(tx_buf);
	}

	void too_many_arguments()
	{
	    int ret = std::sprintf(tx_buf, "--> too many arguments!\r\n");
	    if(0 < ret) cdc_puts(tx_buf);
	}

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
		int ret = std::sprintf(tx_buf, "shirasu version:%s\r\n", SHIRASU_VERSION);
		if(0 < ret) cdc_puts(tx_buf);

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
		else too_many_arguments();
		return 0;
	}

	MSCMD_USER_RESULT usrcmd_diagnostic(MSOPT *msopt, MSCMD_USER_OBJECT usrobj)
	{
		USER_OBJECT *uo = (USER_OBJECT *)usrobj;
		char buf[MSCONF_MAX_INPUT_LENGTH];
		int argc;
		msopt_get_argc(msopt, &argc);
		if(argc == 1){
			switch(control.conf_diag){
				case Diagnostic::usb:
					dump_value("DIAG", "usb");
					break;
				case Diagnostic::cur:
					dump_value("DIAG", "current");
					break;
				case Diagnostic::vel:
					dump_value("DIAG", "velocity");
					break;
				case Diagnostic::pos:
					dump_value("DIAG","position");
					break;
				case Diagnostic::disable:
				default :
					dump_value("DIAG", "disable");
					break;

			}
		}
		else if(argc == 2){
			msopt_get_argv(msopt, 1, buf, sizeof(buf));

			if (std::strcmp(buf, "USB") == 0){
				control.conf_diag = Diagnostic::usb;
				valid_value_set("DIAG", "usb");
			}
			else if(std::strcmp(buf, "CUR") == 0){
				control.conf_diag = Diagnostic::cur;
				valid_value_set("DIAG", "current");
			}
			else if(std::strcmp(buf,"VEL")==0){
				control.conf_diag = Diagnostic::vel;
				valid_value_set("DIAG", "velocity");
			}
			else if(std::strcmp(buf,"POS")==0){
				control.conf_diag = Diagnostic::pos;
				valid_value_set("DIAG", "position");
			}
			else{
				control.conf_diag = Diagnostic::disable;
				valid_value_set("DIAG", "disable");
			}
		}
		else too_many_arguments();
		return 0;
	}

	MSCMD_USER_RESULT usrcmd_error(MSOPT *msopt, MSCMD_USER_OBJECT usrobj)
	{
		USER_OBJECT *uo = (USER_OBJECT *)usrobj;
		char buf[MSCONF_MAX_INPUT_LENGTH];
		int argc;
		msopt_get_argc(msopt, &argc);
		if(argc == 1){
			switch(control.GetError()){
				case Error::out_of_operating_voltage:
					dump_value("ERROR", "out of operating voltage");
					break;
				case Error::out_of_operating_temperature:
					dump_value("ERROR", "out of operating temperature");
					break;
				default:
					dump_value("ERROR", "none");
					break;
			}
		}
		else too_many_arguments();
		return 0;
	}



	MSCMD_USER_RESULT usrcmd_bid(MSOPT *msopt, MSCMD_USER_OBJECT usrobj)
	{
		USER_OBJECT *uo = (USER_OBJECT *)usrobj;
		char buf[MSCONF_MAX_INPUT_LENGTH];
		int argc;
		msopt_get_argc(msopt, &argc);
		char str[256]={};
		if(argc == 1){
			sprintf(str,"--> Current BID: 0X%03x\r\n",control.GetBID());
			uo->puts(str);
		}
		else if(argc == 2){
			msopt_get_argv(msopt, 1, buf, sizeof(buf));
			uint16_t id;
			sscanf(buf,"%3x",&id);
			if(control.SetBID(id)){
				sprintf(str,"--> Invalid value for BID: 0X%03x\r\n",id);
				uo->puts(str);
				cdc_puts("--> BID should be 0x003 < bid < 0x800 and multiples of 4\r\n");
			}
		}
		else too_many_arguments();
		return 0;
	}

	MSCMD_USER_RESULT usrcmd_ppr(MSOPT *msopt, MSCMD_USER_OBJECT usrobj)
	{
		USER_OBJECT *uo = (USER_OBJECT *)usrobj;
		char buf[MSCONF_MAX_INPUT_LENGTH];
		int argc;
		msopt_get_argc(msopt, &argc);

		const char* name = "ppr";
		const char* unit = "Pulse per Revolution";
		if(argc == 1){
		    dump_value(name, unit, control.GetCPR()/4);
		}
		else if(argc == 2){
			msopt_get_argv(msopt, 1, buf, sizeof(buf));
			double ppr;
			std:sscanf(buf,"%lf",&ppr);
			int8_t ret = control.SetCPR(ppr*4);

	        if (ret != 0)
	        {
	            invalid_value(name, ppr);
	        }
	        else
	        {
	            valid_value_set(name, unit, ppr);
	        }
		}
		else too_many_arguments();
		return 0;
	}

	MSCMD_USER_RESULT usrcmd_cpr(MSOPT *msopt, MSCMD_USER_OBJECT usrobj)
	{
		USER_OBJECT *uo = (USER_OBJECT *)usrobj;
		char buf[MSCONF_MAX_INPUT_LENGTH];
		int argc;
		msopt_get_argc(msopt, &argc);

		const char* name = "cpr";
		const char* unit = "Counts per Revolution";
		if(argc == 1){
		    dump_value(name, unit, control.GetCPR());
		}
		else if(argc == 2){
			msopt_get_argv(msopt, 1, buf, sizeof(buf));
			double cpr;
			std:sscanf(buf,"%lf",&cpr);
			int8_t ret = control.SetCPR(cpr);

	        if (ret != 0)
	        {
	            invalid_value(name, cpr);
	        }
	        else
	        {
	            valid_value_set(name, unit, cpr);
	        }
		}
		else too_many_arguments();
		return 0;
	}

	MSCMD_USER_RESULT usrcmd_kpr(MSOPT *msopt, MSCMD_USER_OBJECT usrobj)
	{
		USER_OBJECT *uo = (USER_OBJECT *)usrobj;
		char buf[MSCONF_MAX_INPUT_LENGTH];
		int argc;
		msopt_get_argc(msopt, &argc);
		const char* name = "Kp";
		const char* unit = "A/(rad/s)";
		if(argc == 1){
		    dump_value(name, unit, control.GetKp());
		}
		else if(argc == 2){
			msopt_get_argv(msopt, 1, buf, sizeof(buf));
			double kpr;
			std:sscanf(buf,"%lf",&kpr);
			int8_t ret = control.SetKp(kpr);

	        if (ret != 0)
	        {
	            invalid_value(name, kpr);
	        }
	        else
	        {
	            valid_value_set(name, unit, kpr);
	        }
		}
		else too_many_arguments();
		return 0;
	}

	MSCMD_USER_RESULT usrcmd_kit(MSOPT *msopt, MSCMD_USER_OBJECT usrobj)
	{
		USER_OBJECT *uo = (USER_OBJECT *)usrobj;
		char buf[MSCONF_MAX_INPUT_LENGTH];
		int argc;
		msopt_get_argc(msopt, &argc);

		const char* name = "Ki";
		const char* unit = "A/(rad/s)";
		if(argc == 1){
		    dump_value(name, unit, control.GetKi());
		}
		else if(argc == 2){
			msopt_get_argv(msopt, 1, buf, sizeof(buf));
			double ki;
			std:sscanf(buf,"%lf",&ki);
			int8_t ret = control.SetKi(ki);

	        if (ret != 0)
	        {
	            invalid_value(name, ki);
	        }
	        else
	        {
	            valid_value_set(name, unit, ki);
	        }
		}
		else too_many_arguments();
		return 0;
	}

	MSCMD_USER_RESULT usrcmd_kvp(MSOPT *msopt, MSCMD_USER_OBJECT usrobj)
	{
		USER_OBJECT *uo = (USER_OBJECT *)usrobj;
		char buf[MSCONF_MAX_INPUT_LENGTH];
		int argc;
		msopt_get_argc(msopt, &argc);

		const char* name = "Kv";
		const char* unit = "(rad/s) / rad";
		if(argc == 1){
		    dump_value(name, unit, control.GetKv());
		}
		else if(argc == 2){
			msopt_get_argv(msopt, 1, buf, sizeof(buf));
			double kv;
			std:sscanf(buf,"%lf",&kv);
			int8_t ret = control.SetKv(kv);

	        if (ret != 0)
	        {
	            invalid_value(name, kv);
	        }
	        else
	        {
	            valid_value_set(name, unit, kv);
	        }
		}
		else too_many_arguments();
		return 0;
	}

	MSCMD_USER_RESULT usrcmd_vsp(MSOPT *msopt, MSCMD_USER_OBJECT usrobj)
	{
		USER_OBJECT *uo = (USER_OBJECT *)usrobj;
		char buf[MSCONF_MAX_INPUT_LENGTH];
		int argc;
		msopt_get_argc(msopt, &argc);

		const char* name = "Supply Voltage";
		const char* unit = "V";
		if(argc == 1){
			dump_value(name, unit, control.GetVSP());
		}
		else too_many_arguments();
		return 0;
	}

	MSCMD_USER_RESULT usrcmd_hvl(MSOPT *msopt, MSCMD_USER_OBJECT usrobj)
	{
		USER_OBJECT *uo = (USER_OBJECT *)usrobj;
		char buf[MSCONF_MAX_INPUT_LENGTH];
		int argc;
		msopt_get_argc(msopt, &argc);

		const char* name = "Omega_homing";
		const char* unit = "rad/s";
		if(argc == 1){
		    dump_value(name, unit, control.GetHVL());
		}
		else if(argc == 2){
			msopt_get_argv(msopt, 1, buf, sizeof(buf));
			double hvl;
			std:sscanf(buf,"%lf",&hvl);
			int8_t ret = control.SetHVL(hvl);

	        if (ret != 0)
	        {
	            invalid_value(name, hvl);
	        }
	        else
	        {
	            valid_value_set(name, unit, hvl);
	        }
		}
		else too_many_arguments();
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
				case Mode::duty:
					dump_value("DEF", "duty");
					break;
				case Mode::current:
					dump_value("DEF", "current");
					break;
				case Mode::velocity:
					dump_value("DEF", "disable");
					break;
				case Mode::position:
					dump_value("DEF", "position");
					break;
				default:
					dump_value("DEF", "disable");
					break;
			}
		}
		else if(argc == 2){
			msopt_get_argv(msopt, 1, buf, sizeof(buf));
			if(std::strcmp(buf,"DUT")==0){
				control.SetDefaultMode(Mode::duty);
				valid_value_set("DEF", "duty");
			}
			else if(std::strcmp(buf,"CUR")==0){
				control.SetDefaultMode(Mode::current);
				valid_value_set("DEF", "current");
			}
			else if(std::strcmp(buf,"VEL")==0){
				control.SetDefaultMode(Mode::velocity);
				valid_value_set("DEF", "velocity");
			}
			else if(std::strcmp(buf,"POS")==0){
				control.SetDefaultMode(Mode::position);
				valid_value_set("DEF", "POS");
			}
			else{
				control.SetDefaultMode(Mode::disable);
				valid_value_set("DEF", "disable");
			}
		}
		else too_many_arguments();
		return 0;
	}


	MSCMD_USER_RESULT usrcmd_temp(MSOPT *msopt, MSCMD_USER_OBJECT usrobj)
	{
		USER_OBJECT *uo = (USER_OBJECT *)usrobj;
		char buf[MSCONF_MAX_INPUT_LENGTH];
		int argc;
		msopt_get_argc(msopt, &argc);
		const char* name = "Temperature";
		const char* unit = "degrees";
		if(argc == 1){
			dump_value(name, unit, control.GetTEMP());
		}
		else too_many_arguments();
		return 0;
	}

	MSCMD_USER_RESULT usrcmd_pos(MSOPT *msopt, MSCMD_USER_OBJECT usrobj)
	{
		USER_OBJECT *uo = (USER_OBJECT *)usrobj;
		char buf[MSCONF_MAX_INPUT_LENGTH];
		int argc;
		msopt_get_argc(msopt, &argc);

		const char* name = "Position";
		const char* unit = "rad";
		if(argc == 1){
		    dump_value(name, unit,control.GetPOS());
		}
		else too_many_arguments();
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
			cdc_puts("tick,velocity,target_velocity\r\n");
			control.SetMode(control.GetDefaultMode());
			control.conf_diag = Diagnostic::usb;
			HAL_Delay(3000);
			control.SetTarget(target);
			HAL_Delay(3000);
			control.SetTarget(0);
			control.SetMode(Mode::disable);
			control.conf_diag = Diagnostic::disable;
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
				case Mode::duty:
					dump_value("MODE", "duty");
					break;
				case Mode::current:
					dump_value("MODE","current");
					break;
				case Mode::velocity:
					dump_value("MODE", "velocity");
					break;
				case Mode::position:
					dump_value("MODE", "position");
					break;
				case Mode::homing:
					dump_value("MODE", "homing");
					break;
				case Mode::disable:
					dump_value("MODE", "disable");
					break;
			}
		}
		else if(argc == 2){
			msopt_get_argv(msopt, 1, buf, sizeof(buf));
			if(std::strcmp(buf,"DUT")==0){
				control.SetMode(Mode::duty);//TODO: 成功したかを判定できるように
				valid_value_set("MODE","duty");
			}
			else if(std::strcmp(buf,"CUR")==0){
				control.SetMode(Mode::current);
				valid_value_set("MODE", "current");
			}
			else if(std::strcmp(buf,"VEL")==0){
				control.SetMode(Mode::velocity);
				valid_value_set("MODE", "velocity");
			}
			else if(std::strcmp(buf,"POS")==0){
				control.SetMode(Mode::position);
				valid_value_set("MODE", "position");
			}
			else if(std::strcmp(buf,"HOM")==0){
				control.SetMode(Mode::homing);
				valid_value_set("MODE","homing");
			}
			else if(std::strcmp(buf,"DEF")==0){
				control.SetMode(control.GetDefaultMode());
				valid_value_set("MODE", "default");
			}
			else{
				control.SetMode(Mode::disable);
				valid_value_set("MODE", "disable");
			}
		}
		else too_many_arguments();
		return 0;
	}

	MSCMD_USER_RESULT usrcmd_flash(MSOPT *msopt, MSCMD_USER_OBJECT usrobj)
	{
		control.WriteConfig();
		writeConf();
		cdc_puts("--> written to flash\r\n");
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
		{ 	"HVL" ,   usrcmd_hvl	},
		{   "DEF"    ,   usrcmd_default_mode },
		{	"DIAG", usrcmd_diagnostic	},
		{   "TEST"    ,   usrcmd_test	},
		{   "WCFG"    ,   usrcmd_flash	},
		{   "HELP",     usrcmd_help     },
		{   "?",        usrcmd_help     },
		{ 	"TARGET" ,   usrcmd_target	},
		{ 	"ERROR" ,   usrcmd_error	},
		{ 	"POS" ,   usrcmd_pos	},
	};

}

namespace shell{

	void init(){
		microshell_init(&ms, cdc_put, cdc_getc, action_hook);
		mscmd_init(&mscmd, table, sizeof(table) / sizeof(table[0]), &usrobj);

		int ret = std::sprintf(tx_buf, "\r\nshirasu version:%s\r\n", SHIRASU_VERSION);
		if(0 < ret) cdc_puts(tx_buf);
		switch(control.GetDefaultMode()){
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
				break;
		}
	}

	void update(){
		MSCMD_USER_RESULT r;
		CDC_Transmit_FS(sendbuf,sizeof(sendbuf));
		microshell_getline(&ms, buf, sizeof(buf));
		mscmd_execute(&mscmd, buf, &r);
	}

}


