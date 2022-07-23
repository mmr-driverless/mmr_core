#pragma once
#include <cstdint>

namespace mmr::lib::data {
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
  };
}
