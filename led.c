#include "led.h"

void led1_on(void)
{HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_SET);}
void led1_off(void)
{HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_RESET);}
void led2_on(void)
{HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET);}
void led2_off(void)
{HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET);}
