/*
 * conf.cpp
 *
 *  Created on: Oct 25, 2020
 *      Author: ryuni
 */
#include "flash.hpp"
#include "conf.hpp"

ConfStruct confStruct;

void readConf(void)
{
    loadFlash(DATA_PAGE_ADDR, (uint8_t*)&confStruct, sizeof(ConfStruct));
}

void writeConf(void)
{
    writeFlash(DATA_PAGE_ADDR, (uint16_t*)&confStruct, sizeof(ConfStruct));
}
