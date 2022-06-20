#include <pin.h>
#include "main.h"

struct MmrPin {
  GPIO_TypeDef *gpio;
  uint16_t pin;
};


static MmrPin __gearDown = {
  .gpio = GEAR_CHANGE_GPIO_Port,
  .pin = GEAR_CHANGE_Pin,
};

MmrPin *gearDown = &__gearDown;

void MMR_PIN_Write(MmrPin *pin, MmrPinState state) {
  GPIO_PinState s = state == MMR_PIN_HIGH
    ? GPIO_PIN_SET
    : GPIO_PIN_RESET;

  HAL_GPIO_WritePin(pin->gpio, pin->pin, s);
}

MmrPinState MMR_PIN_Read(MmrPin *pin) {
  return HAL_GPIO_ReadPin(pin->gpio, pin->pin) == GPIO_PIN_SET
    ? MMR_PIN_HIGH
    : MMR_PIN_LOW;
}
