#pragma once
#include <cstdint>
#include <memory.h>

namespace mmr::lib::buffer {
  enum class Encoding {
    BIG_ENDIAN,
    LITTLE_ENDIAN,
  };


  template<Encoding encoding>
  float readFloat(uint8_t *buffer, uint8_t offset);

  template<>
  float readFloat<Encoding::LITTLE_ENDIAN>(uint8_t *buffer, uint8_t offset) {
    return *(float*)(buffer + offset);
  }

  template<>
  float readFloat<Encoding::BIG_ENDIAN>(uint8_t *buffer, uint8_t offset) {
    uint8_t conversionBuffer[] = {
      buffer[offset + 3],
      buffer[offset + 2],
      buffer[offset + 1],
      buffer[offset + 0],
    };
    
    float result;
    memcpy(&result, conversionBuffer, sizeof(float));
    return result;
  }


  template <Encoding encoding>
  constexpr uint16_t readTwo(uint8_t *buffer, uint8_t offset) {
    constexpr uint16_t first = (encoding == Encoding::BIG_ENDIAN ? 0 : 1);
    constexpr uint16_t second = (encoding == Encoding::BIG_ENDIAN ? 1 : 0);

    return buffer[offset + first] << 8 | buffer[offset + second];
  }

  template <Encoding encoding>
  constexpr uint32_t readFour(uint8_t *buffer, uint8_t offset) {
    constexpr uint8_t offset1 = encoding == Encoding::BIG_ENDIAN ? 0 : 2;
    constexpr uint8_t offset2 = encoding == Encoding::BIG_ENDIAN ? 2 : 0;

    constexpr uint32_t first = readTwo<offset, encoding>(buffer, offset + offset1);
    constexpr uint32_t second = readTwo<offset, encoding>(buffer, offset + offset2);

    return first << 16 | second;
  }

  template <Encoding encoding>
  constexpr uint64_t readEight(uint8_t *buffer, uint8_t offset) {
    constexpr uint8_t offset1 = encoding == Encoding::BIG_ENDIAN ? 0 : 4;
    constexpr uint8_t offset2 = encoding == Encoding::BIG_ENDIAN ? 4 : 0;

    constexpr uint64_t first = readFour<offset + offset1, encoding>(buffer, offset + offset1);
    constexpr uint64_t second = readFour<offset + offset2, encoding>(buffer, offset + offset2);

    return first << 32 | second;
  }
}