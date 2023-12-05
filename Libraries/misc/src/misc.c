#include "misc.h"

void gettick_delay_ms(uint32_t delay_time_ms)
{
    uint32_t reference_time = HAL_GetTick();

	while(true){
		if(HAL_GetTick() - reference_time >= delay_time_ms)
			return;
	}

}
