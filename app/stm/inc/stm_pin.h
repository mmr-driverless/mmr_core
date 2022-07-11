#ifndef APP_STM_INC_STM_PIN_H_
#define APP_STM_INC_STM_PIN_H_

#include <pin.h>
#include <stdint.h>
#include <stdbool.h>

#ifndef HAL_GPIO_MODULE_ENABLED
  typedef struct GPIO_TypeDef GPIO_TypeDef;
  typedef enum {
    GPIO_PIN_RESET = 0U,
    GPIO_PIN_SET
  } GPIO_PinState;


  GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
  void HAL_GPIO_WritePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState);
#endif

struct MmrPin {
  GPIO_TypeDef *port;
  uint16_t pin;
  bool hasInvertedLogic;
};


MmrPin MMR_Pin(GPIO_TypeDef *port, uint16_t pin, bool hasInvertedLogic);

#endif // !APP_STM_INC_STM_PIN_H_
