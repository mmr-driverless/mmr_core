#ifndef APP_CORE_BACK_INC_APPS_H_
#define APP_CORE_BACK_INC_APPS_H_

#include <stdint.h>

const uint32_t APPS_MIN = 650;
const uint32_t APPS_SLOPE = 716;

uint32_t MMR_APPS_ComputeSpeed(float percentage);

#endif // !APP_CORE_BACK_INC_APPS_H_
