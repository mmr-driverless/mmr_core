#include "inc/button.h"
#include "../util/inc/util.h"

static void updateReadings(MmrButton *button);
static void updateState(MmrButton *button);


MmrButton MMR_Button(MmrPin *pin) {
  return (MmrButton) {
    .pin = pin,
  };
}


MmrButtonState MMR_BUTTON_Read(MmrButton *button) {
  updateReadings(button);
  updateState(button);

  return button->state;
}


static void updateReadings(MmrButton *button) {
  MmrBitVector8b *readings = &(button->readings);

  *readings <<= 1;
  *readings |= MMR_PIN_Read(button->pin);
}

static void updateState(MmrButton *button) {
  MmrBitVector8b readings = button->readings;
  MmrButtonState *state = &(button->state);

  if (allHigh(readings)) {
    *state = *state != MMR_BUTTON_RELEASED
      ? MMR_BUTTON_JUST_RELEASED
      : MMR_BUTTON_RELEASED;
  }
  else if (allLow(readings)) {
    *state = *state != MMR_BUTTON_PRESSED
      ? MMR_BUTTON_JUST_PRESSED
      : MMR_BUTTON_PRESSED;
  }
}
