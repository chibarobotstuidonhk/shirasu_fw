/*
 * CanClass.cpp
 *
 *  Created on: 2020/10/23
 *      Author: ryuni
 */

#include "CanClass.hpp"
#include "main.h"

extern "C" {
	CAN_HandleTypeDef hcan;
};

CanClass::CanClass()
{
}

void CanClass::init(uint32_t id,uint32_t bitrate)
{
	pclk1 = HAL_RCC_GetPCLK1Freq();
	this->id = id;
    bus_state = OFF_BUS;

	tx_header.RTR = CAN_RTR_DATA;
	tx_header.IDE = CAN_ID_STD;
	tx_header.ExtId = 0;

	can_set_bitrate(bitrate);
	can_set_silent(0);
	can_enable();
}

void CanClass::can_set_filter(uint32_t id, uint32_t mask)
{
    // see page 825 of RM0091 for details on filters
    // set the standard ID part
    filter.FilterIdHigh = (id & 0x7FF) << 5;
    // add the top 5 bits of the extended ID
    filter.FilterIdHigh += (id >> 24) & 0xFFFF;
    // set the low part to the remaining extended ID bits
    filter.FilterIdLow += ((id & 0x1FFFF800) << 3);

    // set the standard ID part
    filter.FilterMaskIdHigh = (mask & 0x7FF) << 5;
    // add the top 5 bits of the extended ID
    filter.FilterMaskIdHigh += (mask >> 24) & 0xFFFF;
    // set the low part to the remaining extended ID bits
    filter.FilterMaskIdLow += ((mask & 0x1FFFF800) << 3);

    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterBank = 0;
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;

    filter.SlaveStartFilterBank = 0;
    filter.FilterActivation = ENABLE;

    if (bus_state == ON_BUS)
    {
        HAL_CAN_ConfigFilter(&hcan, &filter);
    }
}

void CanClass::can_enable(void)
{
    if (bus_state == OFF_BUS)
    {
    	HAL_CAN_DeInit(&hcan);
        hcan.Init.Prescaler = prescaler;
        hcan.Init.Mode = CAN_MODE_NORMAL;

        //Sample-Point at: (1+15)/(1+15+2)=88.9% where CANopen states "The location of the sample point must be as close as possible to 87,5 % of the bit time."
        hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
        hcan.Init.TimeSeg1 = CAN_BS1_15TQ;
        hcan.Init.TimeSeg2 = CAN_BS2_2TQ;

        hcan.Init.TimeTriggeredMode = DISABLE;
        hcan.Init.AutoBusOff = ENABLE;
        hcan.Init.AutoWakeUp = DISABLE;
        hcan.Init.AutoRetransmission = ENABLE;
        hcan.Init.ReceiveFifoLocked = DISABLE;
        hcan.Init.TransmitFifoPriority = DISABLE;

        if (HAL_CAN_Init(&hcan) != HAL_OK)
        {
          Error_Handler();
        }

//        HAL_CAN_Init(&hcan);
        bus_state = ON_BUS;
        can_set_filter(id, 0x07fc);//reserve 4 id

        /* Start the CAN peripheral */
        if (HAL_CAN_Start(&hcan) != HAL_OK)
        {
          /* Start Error */
          Error_Handler();
        }

        /* Activate CAN RX notification */
        if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
        {
          /* Notification Error */
          Error_Handler();
        }
    }
}

void CanClass::can_disable(void)
{
    if (bus_state == ON_BUS)
    {
        // do a bxCAN reset (set RESET bit to 1)
        hcan.Instance->MCR |= CAN_MCR_RESET;
        bus_state = OFF_BUS;
    }
}

void CanClass::can_set_bitrate(uint32_t bitrate)
{
    if (bus_state == ON_BUS)
    {
        // cannot set bitrate while on bus
    	Error_Handler();
        return;
    }

    switch (bitrate)
    {
        case 10000:
        case 20000:
        case 50000:
        case 100000:
        case 125000:
        case 250000:
        case 500000:
        case 1000000:
        	prescaler = pclk1/18/bitrate;
        	break;
        default:
        	Error_Handler();
    }
}

void CanClass::can_set_silent(uint8_t silent)
{
    if (bus_state == ON_BUS)
    {
        // cannot set silent mode while on bus
        return;
    }
    if (silent)
    {
        hcan.Init.Mode = CAN_MODE_SILENT;
    }
    else
    {
        hcan.Init.Mode = CAN_MODE_NORMAL;
    }
}

uint32_t CanClass::can_tx(CAN_TxHeaderTypeDef *tx_header, uint8_t (&buf)[CAN_MTU])
{
    uint32_t status;

    uint32_t tx_mailbox;
    status = HAL_CAN_AddTxMessage(&hcan, tx_header, buf, &tx_mailbox);

    return status;
}

uint32_t CanClass::can_rx(CAN_RxHeaderTypeDef *rx_header, uint8_t (&buf)[CAN_MTU])
{
    uint32_t status;

    status = HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, rx_header, buf);

    return status;
}

uint8_t CanClass::is_can_msg_pending(uint8_t fifo)
{
    if (bus_state == OFF_BUS)
    {
        return 0;
    }
    return (HAL_CAN_GetRxFifoFillLevel(&hcan, fifo) > 0);
}

void CanClass::endit(){
	rx_flag = 0;
	led_on();
}

// Attempt to turn on status LED
void CanClass::led_on(void)
{
	// Make sure the LED has been off for at least LED_DURATION before turning on again
	// This prevents a solid status LED on a busy canbus
	if(led_laston == 0 && HAL_GetTick() - led_lastoff > LED_DURATION)
	{
	    HAL_GPIO_TogglePin(LED_CAN_GPIO_Port, LED_CAN_Pin);
		led_laston = HAL_GetTick();
	}
}

// Process time-based LED events
void CanClass::led_process(void)
{
	// If LED has been on for long enough, turn it off
	if(led_laston > 0 && HAL_GetTick() - led_laston > LED_DURATION)
	{
        HAL_GPIO_TogglePin(LED_CAN_GPIO_Port,LED_CAN_Pin);
		led_laston = 0;
		led_lastoff = HAL_GetTick();
	}
}

void CanClass::Error_Handler(void){

}
