#ifndef APP_LIB_CAN_INC_FILTER_H_
#define APP_LIB_CAN_INC_FILTER_H_

typedef struct MmrCanFilter {
  uint32_t id;
  uint32_t mask;
  bool isExtendedId;
} MmrCanFilter;


MmrCanFilter MMR_CAN_Filter(uint32_t id, uint32_t mask, bool isExtendedId);
MmrCanFilter MMR_CAN_FILTER_SetId(MmrCanFilter *filter, uint32_t id);
MmrCanFilter MMR_CAN_FILTER_GetId(MmrCanFilter *filter, uint32_t id);
MmrCanFilter MMR_CAN_FILTER_SetMask(MmrCanFilter *filter, uint32_t id);
MmrCanFilter MMR_CAN_FILTER_GetMask(MmrCanFilter *filter, uint32_t id);
MmrCanFilter MMR_CAN_FILTER_SetId(MmrCanFilter *filter, uint32_t id);
MmrCanFilter MMR_CAN_FILTER_SetId(MmrCanFilter *filter, uint32_t id);

#endif // !APP_LIB_CAN_INC_FILTER_H_
