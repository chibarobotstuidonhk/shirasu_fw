/*
 * flash.h
 *
 *  Created on: Dec 27, 2018
 *      Author: yusaku
 */

#ifndef FLASH_H_
#define FLASH_H_

#include "main.h"
#include <cstring>

#define DATA_PAGE_ADDR 0x0801FC00

void _eraseFlash(void)
{
    FLASH_EraseInitTypeDef erase;
    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.PageAddress = DATA_PAGE_ADDR;
    erase.NbPages = 1;

    uint32_t pageError = 0;

    HAL_FLASHEx_Erase(&erase, &pageError);
}

void writeFlash(uint32_t address, uint16_t *data, uint32_t size)
{
    HAL_FLASH_Unlock();     /* フラッシュをアンロック */
    _eraseFlash();          /* ページを消去 */
    do {
        /* 2Byteずつフラッシュに書き込む */
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, address, *data);
    } while (address+=2, data++, size-=2);
    HAL_FLASH_Lock();       /* フラッシュをロック */
}

void loadFlash(uint32_t address, uint8_t *data, uint32_t size)
{
    memcpy(data, (uint8_t*)address, size);
}


#endif /* FLASH_H_ */
