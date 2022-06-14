#include "inc/scs.h"

#define EMPTY_HEADER ((MmrCanHeader){})
#define EMPTY_ENTRY ((MmrCanScsEntry){})

static MmrCanScsEntry __scsEntries[MMR_CAN_SCS_ENTRIES_COUNT];


MmrCanScsEntry* MMR_CAN_GetNextScsEntry() {
  static int counter = 0;

  MmrCanScsEntry *entry = __scsEntries + counter;
  counter++;
  counter %= MMR_CAN_SCS_ENTRIES_COUNT;

  return entry;
}


MmrCanScsEntry* MMR_CAN_PutScsEntry(MmrCanHeader header) {
  MmrCanScsEntry *entry = MMR_CAN_FindScsEntry(EMPTY_HEADER);
  if (entry == NULL) {
    return NULL;
  }

  *entry = (MmrCanScsEntry){
    .header = header,
    .counter = MMR_CAN_GetCurrentTick(),
  };

  return entry;
}


MmrCanScsEntry* MMR_CAN_ClearScsEntry(MmrCanHeader header) {
  MmrCanScsEntry *entry = MMR_CAN_FindScsEntry(header);
  if (entry == NULL) {
    return NULL;
  }

  *entry = EMPTY_ENTRY;
  return entry;
}


MmrCanScsEntry* MMR_CAN_FindScsEntry(MmrCanHeader header) {
  uint32_t target = MMR_CAN_HeaderToBits(header);
  uint32_t i = 0;

  for (; i < MMR_CAN_SCS_ENTRIES_COUNT; i++) {
    MmrCanScsEntry *entry = __scsEntries + i;
    uint32_t curr = MMR_CAN_HeaderToBits(entry->header);

    if (curr == target) {
      return entry;
    }
  }

  return NULL;
}
