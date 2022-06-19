#include "buffer.h"

static uint16_t readTwo(uint8_t *buffer, uint8_t offset, MmrEncoding encoding);
static uint32_t readFour(uint8_t *buffer, uint8_t offset, MmrEncoding encoding);
static uint64_t readEight(uint8_t *buffer, uint8_t offset, MmrEncoding encoding);


bool MMR_BUFFER_ReadBool(uint8_t *buffer, uint8_t offset) { return buffer[offset]; }
uint8_t MMR_BUFFER_ReadByte(uint8_t *buffer, uint8_t offset) { return buffer[offset]; }
int16_t MMR_BUFFER_ReadInt16(uint8_t *buffer, uint8_t offset, MmrEncoding encoding) { return readTwo(buffer, offset, encoding); }
uint16_t MMR_BUFFER_ReadUint16(uint8_t *buffer, uint8_t offset, MmrEncoding encoding) { return readTwo(buffer, offset, encoding); }
int32_t MMR_BUFFER_ReadInt32(uint8_t *buffer, uint8_t offset, MmrEncoding encoding) { return readFour(buffer, offset, encoding); }
uint32_t MMR_BUFFER_ReadUint32(uint8_t *buffer, uint8_t offset, MmrEncoding encoding) { return readFour(buffer, offset, encoding); }
int64_t MMR_BUFFER_ReadInt64(uint8_t *buffer, uint8_t offset, MmrEncoding encoding) { return readEight(buffer, offset, encoding); }
uint64_t MMR_BUFFER_ReadUint64(uint8_t *buffer, uint8_t offset, MmrEncoding encoding) { return readEight(buffer, offset, encoding); }
float MMR_BUFFER_ReadFloat(uint8_t *buffer, uint8_t offset, MmrEncoding encoding) { return readFour(buffer, offset, encoding); }


static uint16_t readTwo(uint8_t *buffer, uint8_t offset, MmrEncoding encoding) {
  uint16_t first = offset + (encoding == MMR_ENCODING_BIG_ENDIAN ? 0 : 1);
  uint16_t second = offset + (encoding == MMR_ENCODING_BIG_ENDIAN ? 1 : 0);

  return buffer[first] << 8 | buffer[second];
}

static uint32_t readFour(uint8_t *buffer, uint8_t offset, MmrEncoding encoding) {
  uint32_t first = readTwo(buffer, offset, encoding);
  uint32_t second = readTwo(buffer, offset + 2, encoding);

  return first << 16 | second;
}

static uint64_t readEight(uint8_t *buffer, uint8_t offset, MmrEncoding encoding) {
  uint64_t first = readFour(buffer, offset, encoding);
  uint64_t second = readFour(buffer, offset + 2, encoding);

  return first << 32 | second;
}
