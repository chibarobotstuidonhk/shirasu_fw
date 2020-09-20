/*
 * DWT.hpp
 *
 *  Created on: 2020/09/20
 *      Author: ryuni
 */

#ifndef INC_DWT_HPP_
#define INC_DWT_HPP_

#include "core_cm4.h"

namespace dwt{
	class Tim{
	public:
		Tim(){
			start = DWT->CYCCNT;
		}
		~Tim(){
			if(num < size){
				sum+=DWT->CYCCNT-start;
				++num;
			}
		}
		static void init(){
			CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
			DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
		}
		static uint32_t get_cpu_cycle(){
			return sum/num;
		}
	private:
		static constexpr uint32_t size = 20;
		static uint32_t sum;
		static uint32_t num;
		uint32_t start;
	};
	uint32_t Tim::sum = 0;
	uint32_t Tim::num = 0;
}

#endif /* INC_DWT_HPP_ */
