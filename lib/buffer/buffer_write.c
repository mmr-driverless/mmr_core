#include "buffer.h"
#include <binary_literals.h>

static void writeTwoBytes(uint8_t *buffer, uint8_t *bytes, uint8_t offset, MmrEncoding encoding);
static void writeFourBytes(uint8_t *buffer, uint8_t *bytes, uint8_t offset, MmrEncoding encoding);
static void writeEightBytes(uint8_t *buffer, uint8_t *bytes, uint8_t offset, MmrEncoding encoding);


void MMR_BUFFER_WriteBool(uint8_t *buffer, bool value, uint8_t offset) { buffer[offset] = value; }
void MMR_BUFFER_WriteByte(uint8_t *buffer, uint8_t value, uint8_t offset) { buffer[offset] = value; }
void MMR_BUFFER_WriteInt16(uint8_t *buffer, int16_t value, uint8_t offset, MmrEncoding encoding) { writeTwoBytes(buffer, (uint8_t*)&value, offset, encoding); }
void MMR_BUFFER_WriteUInt16(uint8_t *buffer, uint16_t value, uint8_t offset, MmrEncoding encoding) { writeTwoBytes(buffer, (uint8_t*)&value, offset, encoding); }
void MMR_BUFFER_WriteInt32(uint8_t *buffer, int32_t value, uint8_t offset, MmrEncoding encoding) { writeFourBytes(buffer, (uint8_t*)&value, offset, encoding); }
void MMR_BUFFER_WriteUInt32(uint8_t *buffer, uint32_t value, uint8_t offset, MmrEncoding encoding) { writeFourBytes(buffer, (uint8_t*)&value, offset, encoding); }
void MMR_BUFFER_WriteInt64(uint8_t *buffer, uint64_t value, uint8_t offset, MmrEncoding encoding) { writeEightBytes(buffer, (uint8_t*)&value, offset, encoding); }
void MMR_BUFFER_WriteUint64(uint8_t *buffer, uint64_t value, uint8_t offset, MmrEncoding encoding) { writeEightBytes(buffer, (uint8_t*)&value, offset, encoding); }
void MMR_BUFFER_WriteFloat(uint8_t *buffer, float value, uint8_t offset, MmrEncoding encoding) { writeFourBytes(buffer, (uint8_t*)&value, offset, encoding); }


static void writeTwoBytes(uint8_t *buffer, uint8_t *bytes, uint8_t offset, MmrEncoding encoding) {
  uint8_t first = encoding == MMR_ENCODING_BIG_ENDIAN ? 0 : 1;
  uint8_t second = encoding == MMR_ENCODING_BIG_ENDIAN ? 1 : 0;

  buffer[offset] = bytes[first];
  buffer[offset + 1] = bytes[second];
}

static void writeFourBytes(uint8_t *buffer, uint8_t *bytes, uint8_t offset, MmrEncoding encoding) {
  writeTwoBytes(buffer, bytes, offset, encoding);
  writeTwoBytes(buffer, bytes + 2, offset, encoding);
}

static void writeEightBytes(uint8_t *buffer, uint8_t *bytes, uint8_t offset, MmrEncoding encoding) {
  writeFourBytes(buffer, bytes, offset, encoding);
  writeFourBytes(buffer, bytes + 4, offset, encoding);
}
