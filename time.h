
#ifndef TIME_H_
#define TIME_H_

#include "stm32f10x.h"
#include "i2c_lcd.h"
#include <stdint.h>
#include <stdio.h>

void Time(uint8_t u8Second, uint8_t u8Minute, uint8_t u8Hour, uint8_t u8Day, uint8_t u8Month, uint8_t u8Year);


#endif /* TIME_H_ */
