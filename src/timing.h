#ifndef Timing_h
#define Timing_h

#include "stm32f0xx.h"
#include "stm32f0xx_hal.h"

void initTiming();

inline void delayTicks(uint8_t ticks) {
	volatile uint32_t start = TIM3->CNT;
	while((TIM3->CNT-start) <= ticks);
}
void delayMicros(uint32_t micros);


#endif // Timing_h
