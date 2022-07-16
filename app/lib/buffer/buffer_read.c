#include "buffer.h"

static uint16_t readTwo(uint8_t *buffer, uint8_t offset, MmrEncoding encoding);
static uint32_t readFour(uint8_t *buffer, uint8_t offset, MmrEncoding encoding);
static uint8_t *readFourPtr(uint8_t *buffer, uint8_t offset, MmrEncoding encoding);
static uint64_t readEight(uint8_t *buffer, uint8_t offset, MmrEncoding encoding);


bool MMR_BUFFER_ReadBool(uint8_t *buffer, uint8_t offset) { return buffer[offset]; }
uint8_t MMR_BUFFER_ReadByte(uint8_t *buffer, uint8_t offset) { return buffer[offset]; }
int16_t MMR_BUFFER_ReadInt16(uint8_t *buffer, uint8_t offset, MmrEncoding encoding) { return readTwo(buffer, offset, encoding); }
uint16_t MMR_BUFFER_ReadUint16(uint8_t *buffer, uint8_t offset, MmrEncoding encoding) { return readTwo(buffer, offset, encoding); }
int32_t MMR_BUFFER_ReadInt32(uint8_t *buffer, uint8_t offset, MmrEncoding encoding) { return readFour(buffer, offset, encoding); }
uint32_t MMR_BUFFER_ReadUint32(uint8_t *buffer, uint8_t offset, MmrEncoding encoding) { return readFour(buffer, offset, encoding); }
int64_t MMR_BUFFER_ReadInt64(uint8_t *buffer, uint8_t offset, MmrEncoding encoding) { return readEight(buffer, offset, encoding); }
uint64_t MMR_BUFFER_ReadUint64(uint8_t *buffer, uint8_t offset, MmrEncoding encoding) { return readEight(buffer, offset, encoding); }
float MMR_BUFFER_ReadFloat(uint8_t *buffer, uint8_t offset, MmrEncoding encoding) { return *(float*)readFourPtr(buffer, offset, encoding); }


static uint16_t readTwo(uint8_t *buffer, uint8_t offset, MmrEncoding encoding) {
  uint16_t first = offset + (encoding == MMR_ENCODING_BIG_ENDIAN ? 0 : 1);
  uint16_t second = offset + (encoding == MMR_ENCODING_BIG_ENDIAN ? 1 : 0);

  return buffer[first] << 8 | buffer[second];
}

static uint32_t readFour(uint8_t *buffer, uint8_t offset, MmrEncoding encoding) {
  uint8_t offset1 = encoding == MMR_ENCODING_BIG_ENDIAN ? 0 : 2;
  uint8_t offset2 = encoding == MMR_ENCODING_BIG_ENDIAN ? 2 : 0;

  uint32_t first = readTwo(buffer, offset + offset1, encoding);
  uint32_t second = readTwo(buffer, offset + offset2, encoding);

  return first << 16 | second;
}

static uint8_t *readFourPtr(uint8_t *buffer, uint8_t offset, MmrEncoding encoding) {
  static uint8_t conversionBuffer[4] = {};

  conversionBuffer[3] = buffer[offset + 3];
  conversionBuffer[2] = buffer[offset + 2];
  conversionBuffer[1] = buffer[offset + 1];
  conversionBuffer[0] = buffer[offset + 0];
  return conversionBuffer;
}

static uint64_t readEight(uint8_t *buffer, uint8_t offset, MmrEncoding encoding) {
  uint8_t offset1 = encoding == MMR_ENCODING_BIG_ENDIAN ? 0 : 4;
  uint8_t offset2 = encoding == MMR_ENCODING_BIG_ENDIAN ? 4 : 0;

  uint64_t first = readFour(buffer, offset + offset2, encoding);
  uint64_t second = readFour(buffer, offset + offset1, encoding);

  return first << 32 | second;
}
