#ifndef __ARDUINO_FUNCTIONS_H__
#define __ARDUINO_FUNCTIONS_H__

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>
void togglePin(GPIO_TypeDef* port,uint16_t pin)
{
    HAL_GPIO_TogglePin(port,pin);
}

void digitalWrite(GPIO_TypeDef* port,uint16_t pin, bool state)
{
    HAL_GPIO_WritePin(port,pin,state == true ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

bool digitalRead(GPIO_TypeDef* port, uint16_t pin)
{
    return (HAL_GPIO_ReadPin(port,pin) == GPIO_PIN_SET ? true : false);
}

uint32_t millis()
{
    return HAL_GetTick();
}
void delay(uint32_t time)
{
    HAL_Delay(time);
}

long map(long x, long in_min, long in_max, long out_min, long out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


#endif // __ARDUINO_FUNCTIONS_H__