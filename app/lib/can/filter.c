#include "inc/filter.h"


MmrCanFilter MMR_CAN_Filter(uint32_t id, uint32_t mask, bool isExtendedId) {
  return (MmrCanFilter) {
    .id = id,
    .mask = mask,
    .isExtendedId = isExtendedId,
  };
}

void MMR_CAN_FILTER_SetId(MmrCanFilter *filter, uint32_t id) {
  filter->id = id;
}

uint32_t MMR_CAN_FILTER_GetId(MmrCanFilter *filter) {
  return filter->id;
}

void MMR_CAN_FILTER_SetMask(MmrCanFilter *filter, uint32_t mask) {
  filter->mask = mask;
}

uint32_t MMR_CAN_FILTER_GetMask(MmrCanFilter *filter) {
  return filter->mask;
}

void MMR_CAN_FILTER_SetIsExtendedId(MmrCanFilter *filter, bool isExtendedId) {
  filter->isExtendedId = isExtendedId;
}

bool MMR_CAN_FILTER_IsExtendedId(MmrCanFilter *filter) {
  return filter->isExtendedId;
}
