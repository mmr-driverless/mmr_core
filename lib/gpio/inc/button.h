#ifndef LIB_GPIO_INC_BUTTON_H_
#define LIB_GPIO_INC_BUTTON_H_

#include "pin.h"

typedef struct MmrButton MmrButton;

typedef enum MmrButtonState {
  MMR_BUTTON_RELEASED,
  MMR_BUTTON_PRESSED,
  MMR_BUTTON_JUST_RELEASED,
  MMR_BUTTON_JUST_PRESSED,
} MmrButtonState;


MmrButton MMR_Button(MmrPin *pin);
MmrButtonState MMR_BUTTON_Read(MmrButton *button);

#endif // !LIB_GPIO_INC_BUTTON_H_
