#ifndef LIB_GPIO_INC_PIN_H_
#define LIB_GPIO_INC_PIN_H_

typedef struct MmrPin MmrPin;

typedef enum MmrPinState {
  MMR_PIN_LOW,
  MMR_PIN_HIGH,
} MmrPinState;


void MMR_PIN_Write(MmrPin *pin, MmrPinState state);
MmrPinState MMR_PIN_Read(MmrPin *pin);

#endif // !LIB_GPIO_INC_PIN_H_
