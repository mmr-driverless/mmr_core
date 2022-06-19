#ifndef BUFFER_BUFFER_H_
#define BUFFER_BUFFER_H_

#include <stdint.h>
#include <stdbool.h>

typedef enum MmrEncoding {
  MMR_ENCODING_BIG_ENDIAN,
  MMR_ENCODING_LITTLE_ENDIAN,
} MmrEncoding;


void MMR_BUFFER_WriteBool(uint8_t *buffer, bool value, uint8_t offset);
void MMR_BUFFER_WriteByte(uint8_t *buffer, uint8_t value, uint8_t offset);
void MMR_BUFFER_WriteInt16(uint8_t *buffer, int16_t value, uint8_t offset, MmrEncoding encoding);
void MMR_BUFFER_WriteUInt16(uint8_t *buffer, uint16_t value, uint8_t offset, MmrEncoding encoding);
void MMR_BUFFER_WriteInt32(uint8_t *buffer, int32_t value, uint8_t offset, MmrEncoding encoding);
void MMR_BUFFER_WriteUInt32(uint8_t *buffer, uint32_t value, uint8_t offset, MmrEncoding encoding);
void MMR_BUFFER_WriteInt64(uint8_t *buffer, uint64_t value, uint8_t offset, MmrEncoding encoding);
void MMR_BUFFER_WriteUint64(uint8_t *buffer, uint64_t value, uint8_t offset, MmrEncoding encoding);
void MMR_BUFFER_WriteFloat(uint8_t *buffer, float value, uint8_t offset, MmrEncoding encoding);

bool MMR_BUFFER_ReadBool(uint8_t *buffer, uint8_t offset);
uint8_t MMR_BUFFER_ReadByte(uint8_t *buffer, uint8_t offset);
int16_t MMR_BUFFER_ReadInt16(uint8_t *buffer, uint8_t offset, MmrEncoding encoding);
uint16_t MMR_BUFFER_ReadUint16(uint8_t *buffer, uint8_t offset, MmrEncoding encoding);
int32_t MMR_BUFFER_ReadInt32(uint8_t *buffer, uint8_t offset, MmrEncoding encoding);
uint32_t MMR_BUFFER_ReadUint32(uint8_t *buffer, uint8_t offset, MmrEncoding encoding);
int64_t MMR_BUFFER_ReadInt64(uint8_t *buffer, uint8_t offset, MmrEncoding encoding);
uint64_t MMR_BUFFER_ReadUint64(uint8_t *buffer, uint8_t offset, MmrEncoding encoding);
float MMR_BUFFER_ReadFloat(uint8_t *buffer, uint8_t offset, MmrEncoding encoding);

#endif // !BUFFER_BUFFER_H_