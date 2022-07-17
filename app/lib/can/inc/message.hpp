#pragma once
#include "message_id.hpp"
#include "header.hpp"

constexpr uint8_t MMR_CAN_MAX_DATA_LENGTH = 8;
using Buffer = uint8_t[MMR_CAN_MAX_DATA_LENGTH];

namespace mmr::can {
  struct Message {
    constexpr Message():
      id(0),
      isStandardId(false),
      payload({}),
      length(8)
    {};

    constexpr Message(Header header):
      id(header.toBits()),
      isStandardId(false),
      payload({}),
      length(8)
    {}

    const uint32_t id;
    const bool isStandardId;
    const Buffer payload;
    const uint8_t length;
  };
}
