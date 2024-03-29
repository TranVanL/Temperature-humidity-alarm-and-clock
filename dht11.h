#ifndef DHT11_H_
#define DHT11_H_

#include "stm32f10x.h"

void dht11_init(void);
uint8_t dht11_read(uint8_t *pu8Data);

#endif
