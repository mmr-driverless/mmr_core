#include <pin_out.h>
#include "main.h"

struct MmrPin {
  GPIO_TypeDef *port;
  uint16_t pin;
};


static MmrPin __gearDown = {
  .port = GEAR_CHANGE_GPIO_Port,
  .pin = GEAR_CHANGE_Pin,
};

static MmrPin __changeMode = {
  .port = BUTTON_GPIO_Port,
  .pin = BUTTON_Pin,
};

MmrPin *gearDown = &__gearDown;
MmrPin *changeMode = &__changeMode;


void MMR_PIN_Write(MmrPin *pin, MmrPinState state) {
  GPIO_PinState s = state == MMR_PIN_HIGH
    ? GPIO_PIN_SET
    : GPIO_PIN_RESET;

  HAL_GPIO_WritePin(pin->port, pin->pin, s);
}

MmrPinState MMR_PIN_Read(MmrPin *pin) {
  return HAL_GPIO_ReadPin(pin->port, pin->pin) == GPIO_PIN_SET
    ? MMR_PIN_HIGH
    : MMR_PIN_LOW;
}
