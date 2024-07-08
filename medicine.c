#include "medicine.h"

int get_medicine(void)
{
	return HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_2);
}
