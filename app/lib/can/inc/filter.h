#ifndef APP_LIB_CAN_INC_FILTER_H_
#define APP_LIB_CAN_INC_FILTER_H_

#include <stdint.h>
#include <stdbool.h>

typedef struct MmrCanFilter {
  uint32_t id;
  uint32_t mask;
  bool isExtendedId;
} MmrCanFilter;


MmrCanFilter MMR_CAN_Filter(uint32_t id, uint32_t mask, bool isExtendedId);

void MMR_CAN_FILTER_SetId(MmrCanFilter *filter, uint32_t id);
uint32_t MMR_CAN_FILTER_GetId(MmrCanFilter *filter);
void MMR_CAN_FILTER_SetMask(MmrCanFilter *filter, uint32_t mask);
uint32_t MMR_CAN_FILTER_GetMask(MmrCanFilter *filter);
void MMR_CAN_FILTER_SetIsExtendedId(MmrCanFilter *filter, bool isExtendedId);
bool MMR_CAN_FILTER_IsExtendedId(MmrCanFilter *filter);

#endif // !APP_LIB_CAN_INC_FILTER_H_
