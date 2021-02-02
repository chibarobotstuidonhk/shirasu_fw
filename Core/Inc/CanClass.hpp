/*
 * CanClass.hpp
 *
 *  Created on: 2020/10/23
 *      Author: ryuni
 */

#ifndef CANCLASS_HPP_
#define CANCLASS_HPP_

#include "main.h"

namespace{
	enum can_bus_state {
		OFF_BUS,
		ON_BUS
	};

	template<typename T>
	union _Encapsulator
	{
		T data;
		uint64_t i;
	};
}


class CanClass {
public:
	CanClass();
	template<typename T>
	void send(T data,uint32_t id);
	template<typename T>
	bool receive(T &data,uint32_t id);
	void init(uint32_t id,uint32_t bitrate=1000000);
	void endit(void);
	void led_on(void);
	void led_process(void);
private:
//	CAN_HandleTypeDef hcan;
	CAN_TxHeaderTypeDef tx_header;
	CAN_RxHeaderTypeDef rx_header;
	static constexpr uint8_t CAN_MTU = 8;
	uint8_t tx_payload[CAN_MTU];
	uint8_t rx_payload[CAN_MTU];
	uint32_t id;
	uint32_t status;
	uint8_t rx_flag = 0;
	static constexpr uint16_t LED_DURATION = 10;
	uint32_t led_laston = 0;
	uint32_t led_lastoff = 0;
	CAN_FilterTypeDef filter;
	uint32_t pclk1;
	uint32_t prescaler;
	enum can_bus_state bus_state;
	void can_set_filter(uint32_t id, uint32_t mask);
	void can_enable(void);
	void can_disable(void);
	void can_set_bitrate(uint32_t bitrate);
	void can_set_silent(uint8_t silent);
	uint32_t can_tx(CAN_TxHeaderTypeDef *tx_header, uint8_t (&buf)[CAN_MTU]);
	uint32_t can_rx(CAN_RxHeaderTypeDef *rx_header, uint8_t (&buf)[CAN_MTU]);
	uint8_t is_can_msg_pending(uint8_t fifo);
	void Error_Handler(void);
};

template<typename T>
void CanClass::send(T data,uint32_t id)
{
	if (bus_state == OFF_BUS) return;
    _Encapsulator<T> _e;
    _e.data = data;

    for (uint8_t i = sizeof(T); i > 0;)
    {
        i--;
        tx_payload[i] = _e.i & 0xff;
        _e.i >>= 8;
    }
    tx_header.StdId = id;
    tx_header.DLC = sizeof(T);
    can_tx(&tx_header, tx_payload);
}

template<typename T>
bool CanClass::receive(T &data,uint32_t id)
{

	status = can_rx(&rx_header, rx_payload);

	if (status == HAL_OK or rx_flag == 1)
	{
		if(rx_header.StdId == id){
			_Encapsulator<T> _e;

			for (uint8_t i = 0; i < sizeof(T); i++)
			{
				_e.i = (_e.i << 8) | (uint64_t) (rx_payload[i]);
			}

			data = _e.data;
			return true;
		}
		else{
			rx_flag=1;
		}
	}
	return false;
}

#endif /* CANCLASS_HPP_ */
