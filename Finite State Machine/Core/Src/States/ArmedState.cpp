#include "States/ArmedState.h"
#include "stm32f1xx_hal.h"

void ArmedState::onUpdate() {
	HAL_Delay(500);
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
}
