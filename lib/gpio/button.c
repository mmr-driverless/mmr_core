#include "inc/button.h"

static void updateReadings(MmrButton *button);
static void updateState(MmrButton *button);


typedef struct MmrButton {
  MmrPin *pin;
  MmrBitVector8b readings;
  MmrButtonState state;
} MmrButton;


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
  MmrBitVector8b *readings = &(button->_readings);

  *readings <<= 1;
  *readings |= MMR_PIN_Read(button->pin);
}

static void updateState(MmrButton *button) {
  MmrBitVector8b readings = button->_readings;
  MmrButtonState *state = &(button->_state);

  if (allHigh(readings)) {
    *state = *state != RELEASED
      ? JUST_RELEASED
      : RELEASED;
  }
  else if (allLow(readings)) {
    *state = *state != PRESSED
      ? JUST_PRESSED
      : PRESSED;
  }
}
