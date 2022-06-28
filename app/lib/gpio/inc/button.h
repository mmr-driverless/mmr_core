#ifndef LIB_GPIO_INC_BUTTON_H_
#define LIB_GPIO_INC_BUTTON_H_

#include <util.h>
#include <pin.h>


typedef enum MmrButtonState {
  MMR_BUTTON_RELEASED,
  MMR_BUTTON_PRESSED,
  MMR_BUTTON_JUST_RELEASED,
  MMR_BUTTON_JUST_PRESSED,
} MmrButtonState;

typedef struct MmrButton {
  MmrPin *pin;
  uint16_t readings;
  MmrButtonState state;
} MmrButton;


MmrButton MMR_Button(MmrPin *pin);
MmrButtonState MMR_BUTTON_Read(MmrButton *button);

#endif // !LIB_GPIO_INC_BUTTON_H_
