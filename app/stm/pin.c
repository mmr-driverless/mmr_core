#include "inc/stm_pin.h"

static GPIO_PinState mmrToGpio(MmrPin *pin, MmrPinState state);
static MmrPinState gpioToMmr(MmrPin *pin, GPIO_PinState state);


MmrPin MMR_Pin(GPIO_TypeDef *port, uint16_t pin, bool hasInvertedLogic) {
  return (MmrPin) {
    .port = port,
    .pin = pin,
    .hasInvertedLogic = hasInvertedLogic,
  };
}

void MMR_PIN_Write(MmrPin *pin, MmrPinState state) {
  GPIO_PinState s = mmrToGpio(pin, state);
  HAL_GPIO_WritePin(pin->port, pin->pin, s);
}

MmrPinState MMR_PIN_Read(MmrPin *pin) {
  GPIO_PinState state = HAL_GPIO_ReadPin(pin->port, pin->pin);
  return gpioToMmr(pin, state);
}


static GPIO_PinState mmrToGpio(MmrPin *pin, MmrPinState state) {
  if (pin->hasInvertedLogic) {
    return state == MMR_PIN_HIGH
      ? GPIO_PIN_RESET
      : GPIO_PIN_SET;
  }

  return state == MMR_PIN_HIGH
    ? GPIO_PIN_SET
    : GPIO_PIN_RESET;
}

static MmrPinState gpioToMmr(MmrPin *pin, GPIO_PinState state) {
  if (pin->hasInvertedLogic) {
    return state == GPIO_PIN_SET
      ? MMR_PIN_LOW
      : MMR_PIN_HIGH;
  }

  return state == GPIO_PIN_SET
    ? MMR_PIN_HIGH
    : MMR_PIN_LOW;
}
