#pragma once
#include <cstdint>

namespace mmr::lib::data {
  enum class Encoding {
    BIG_ENDIAN,
    LITTLE_ENDIAN,
  };


  template <Encoding encoding>
  class Buffer {
  public:
    constexpr Buffer(uint8_t *data)
      : data(data)
    {}

    constexpr Buffer& writeBool(bool value, uint8_t offset) {  }
    constexpr Buffer& writeByte(uint8_t value, uint8_t offset) {  }
    constexpr Buffer& writeInt16(int16_t value, uint8_t offset) {  }
    constexpr Buffer& writeUInt16(uint16_t value, uint8_t offset) {  }
    constexpr Buffer& writeInt32(int32_t value, uint8_t offset) {  }
    constexpr Buffer& writeUInt32(uint32_t value, uint8_t offset) {  }
    constexpr Buffer& writeInt64(uint64_t value, uint8_t offset) {  }
    constexpr Buffer& writeUint64(uint64_t value, uint8_t offset) {  }
    constexpr Buffer& writeFloat(float value, uint8_t offset) {  }

    [[nodiscard]] constexpr bool readBool(uint8_t offset) const noexcept { return data[offset]; }
    [[nodiscard]] constexpr uint8_t readByte(uint8_t offset) const noexcept { return data[offset]; }
    [[nodiscard]] constexpr int16_t readInt16(uint8_t offset) const noexcept { return readTwo(offset); }
    [[nodiscard]] constexpr uint16_t readUint16(uint8_t offset) const noexcept { return readTwo(offset); }
    [[nodiscard]] constexpr int32_t readInt32(uint8_t offset) const noexcept { return readFour(offset); }
    [[nodiscard]] constexpr uint32_t readUint32(uint8_t offset) const noexcept { return readFour(offset); }
    [[nodiscard]] constexpr int64_t readInt64(uint8_t offset) const noexcept { return readEight(offset); }
    [[nodiscard]] constexpr uint64_t readUint64(uint8_t offset) const noexcept { return readEight(offset); }
    [[nodiscard]] constexpr float readFloat(uint8_t offset) const noexcept { return *(float*)readFourPtr(offset); }

  private:
    uint8_t *data;

    [[nodiscard]]
    constexpr uint16_t readTwo<Encoding::BIG_ENDIAN>(uint8_t offset) const noexcept {
      return data[0] << 8 | data[1];
    }

    [[nodiscard]]
    constexpr uint16_t readTwo<Encoding::LITTLE_ENDIAN>(uint8_t offset) const noexcept {
      return data[1] << 8 | data[0];
    }

    [[nodiscard]]
    constexpr uint32_t readFour<Encoding::BIG_ENDIAN>(uint8_t offset) const noexcept {
      uint32_t first = readTwo<Encoding::BIG_ENDIAN>(data, offset + 0);
      uint32_t second = readTwo<Encoding::BIG_ENDIAN>(data, offset + 2);

      return first << 16 | second;
    }

    [[nodiscard]]
    constexpr uint32_t readFour<Encoding::LITTLE_ENDIAN>(uint8_t offset) const noexcept {
      uint32_t first = readTwo<Encoding::LITTLE_ENDIAN>(data, offset + 2);
      uint32_t second = readTwo<Encoding::LITTLE_ENDIAN>(data, offset + 0);

      return first << 16 | second;
    }


    [[nodiscard]]
    constexpr uint8_t *readFourPtr(uint8_t offset) const noexcept {
     uint8_t conversionBuffer[4] = {};

      conversionBuffer[3] = data[offset + 3];
      conversionBuffer[2] = data[offset + 2];
      conversionBuffer[1] = data[offset + 1];
      conversionBuffer[0] = data[offset + 0];
      return conversionBuffer;
    }

    [[nodiscard]]
    constexpr uint64_t readEight<Encoding::BIG_ENDIAN>(uint8_t offset) const noexcept {
      uint64_t first = readFour<Encoding::BIG_ENDIAN>(data, offset + 0);
      uint64_t second = readFour<Encoding::BIG_ENDIAN>(data, offset + 4);

      return first << 32 | second;
    }

    [[nodiscard]]
    constexpr uint64_t readEight<Encoding::LITTLE_ENDIAN>(uint8_t offset) const noexcept {
      uint64_t first = readFour<Encoding::LITTLE_ENDIAN>(data, offset + 4);
      uint64_t second = readFour<Encoding::LITTLE_ENDIAN>(data, offset + 0);

      return first << 32 | second;
    }



static void writeTwoBytes(uint8_t *buffer, uint8_t *bytes, uint8_t offset, MmrEncoding encoding) {
  uint8_t first = encoding == MMR_ENCODING_BIG_ENDIAN ? 0 : 1;
  uint8_t second = encoding == MMR_ENCODING_BIG_ENDIAN ? 1 : 0;

  buffer[offset] = bytes[first];
  buffer[offset + 1] = bytes[second];
}

static void writeFourBytes(uint8_t *buffer, uint8_t *bytes, uint8_t offset, MmrEncoding encoding) {
  uint8_t offset1 = encoding == MMR_ENCODING_BIG_ENDIAN ? 0 : 2;
  uint8_t offset2 = encoding == MMR_ENCODING_BIG_ENDIAN ? 2 : 0;

  writeTwoBytes(buffer, bytes, offset + offset1, encoding);
  writeTwoBytes(buffer, bytes, offset + offset2, encoding);
}

static void writeEightBytes(uint8_t *buffer, uint8_t *bytes, uint8_t offset, MmrEncoding encoding) {
  uint8_t offset1 = encoding == MMR_ENCODING_BIG_ENDIAN ? 0 : 4;
  uint8_t offset2 = encoding == MMR_ENCODING_BIG_ENDIAN ? 4 : 0;

  writeFourBytes(buffer, bytes, offset + offset1, encoding);
  writeFourBytes(buffer, bytes, offset + offset2, encoding);
}

  };
}
