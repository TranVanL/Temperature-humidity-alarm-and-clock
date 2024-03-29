#include "time.h"

void Time(uint8_t u8Second, uint8_t u8Minute, uint8_t u8Hour, uint8_t u8Day, uint8_t u8Month, uint8_t u8Year){
	if (u8Second == 60U){
		u8Second = 0U;
		u8Minute++;
		if (u8Minute == 60U){
			u8Minute = 0U;
			u8Hour++;
			if (u8Hour == 24U){
				u8Hour = 0U;
				u8Day++;
				if (u8Day == 29U && u8Month == 2){
					u8Day = 0;
					u8Month++;
				}
				else if (u8Day == 30U && ((u8Month == 4) | (u8Month == 6) | (u8Month == 9) | (u8Month == 11) )){
					u8Day = 0;
					u8Month++;


			   }
				else if (u8Day == 31U){
					u8Day = 0;
					u8Month++;
					if (u8Month == 12U){
						u8Month = 0U;
						u8Year++;

					}

					}
				}


		}


	}


}
