#ifndef APP_CORE_BACK_INC_APPS_H_
#define APP_CORE_BACK_INC_APPS_H_

#include <stdint.h>
#include <stdbool.h>

void MMR_APPS_TryWrite(uint32_t value);
bool MMR_APPS_Check();
uint32_t MMR_APPS_ComputeSpeed(float percentage);

#endif // !APP_CORE_BACK_INC_APPS_H_
