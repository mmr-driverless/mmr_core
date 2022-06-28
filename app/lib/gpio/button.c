#include "inc/button.h"
#include "../util/inc/util.h"


MmrButton MMR_Button(MmrPin *pin) {
  return (MmrButton) {
    .pin = pin,
  };
}


MmrButtonState MMR_BUTTON_Read(MmrButton *button) {
  button->readings <<= 1;
  button->readings |= MMR_PIN_Read(button->pin);

  if (button->readings == 0xFFFF) {
    if (button->state != MMR_BUTTON_RELEASED) {
      button->state = MMR_BUTTON_RELEASED;
      return MMR_BUTTON_JUST_RELEASED;
    }
  }
  else if (button->readings == 0x0000) {
    if (button->state != MMR_BUTTON_PRESSED) {
      button->state = MMR_BUTTON_PRESSED;
      return MMR_BUTTON_JUST_PRESSED;
    }
  }

  return button->state;
}
