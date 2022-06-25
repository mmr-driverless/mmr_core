#ifndef LIB_TIMING_INC_TIMING_H_
#define LIB_TIMING_INC_TIMING_H_

#include <stdint.h>

typedef uint32_t Tick;
typedef Tick (*TickProvider)();


void MMR_SetTickProvider(TickProvider tickProvider);
Tick MMR_GetTick();

#endif // !LIB_TIMING_INC_TIMING_H_
