#ifndef APP_CORE_INC_GEAR_CHANGE_H_
#define APP_CORE_INC_GEAR_CHANGE_H_

#include <pin.h>

void MMR_GEAR_CHANGE_Init(MmrPin *gearUp, MmrPin *gearDn);
void MMR_GEAR_CHANGE_Run();

#endif // !APP_CORE_INC_GEAR_CHANGE_H_
