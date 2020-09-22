/*
 * DWT.hpp
 *
 *  Created on: 2020/09/20
 *      Author: ryuni
 */

#ifndef INC_DWT_HPP_
#define INC_DWT_HPP_

#include "core_cm4.h"
#include "system_stm32f3xx.h"

namespace dwt{
	static void init(){
		CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
		DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
	}

	class ProcessTim{
	public:
		ProcessTim(){
			start = DWT->CYCCNT;
		}
		~ProcessTim(){
			uint32_t stop = DWT->CYCCNT-start;
			if(num < size){
				sum += stop;
				++num;
			}
		}

		static float get_process_time(){
			return (float)sum/num/SystemCoreClock*1000;
		}
	private:
		static constexpr uint32_t size = 100;
		static inline uint32_t sum = 0;
		static inline uint32_t num = 0;
		uint32_t start;
	};

	class Frequency{
	public:
		Frequency(){
			if(last == 0 || num > size) last = DWT->CYCCNT;
			else{
				uint32_t now = DWT->CYCCNT;
				sum += now - last;
				last = now;
				++num;
			}
		}
		static float get_process_frequency(){
			if(sum == 0){
				return 0;
			}
			else{
				return (float)SystemCoreClock*(float)num/(float)sum;
			}
		}
	private:
		static constexpr uint32_t size = 100;
		static inline uint32_t sum = 0;
		static inline uint32_t num = 0;
		static inline uint32_t last = 0;
	};
}

#endif /* INC_DWT_HPP_ */
