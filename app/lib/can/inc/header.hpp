/**
 * @file mmr_can_header.h
 * @brief
 * This file defines the header used for the can message,
 * along with its utilities.
 * 
 * With header is intended the ExtendedId portion
 * of the can message.
 */
#pragma once
#include "message_id.hpp"

#include <stdbool.h>
#include <util.h>


namespace mmr::can {
  enum class MessageType {
    SCS = 0b000,
    ACK = 0b001,
    MULTI_FRAME = 0b010,
    MULTI_FRAME_END = 0b011,
    NORMAL = 0b100,
  };

  enum class MessagePriority {
    LOW = 0b010,
    NORMAL = 0b001,
    HIGH = 0b000,
  };


  struct Header {
    MessageType messageType : 3;
    MessagePriority priority : 3;
    uint8_t seqNumber : 3;
    uint16_t senderId : 10;
    uint16_t messageId : 10;


    [[nodiscard]]
    constexpr uint32_t toBits() const noexcept {
      return 0
        | ((uint32_t)messageType << 26)
        | ((uint32_t)priority << 23)
        | ((uint32_t)seqNumber << 20)
        | ((uint32_t)senderId << 10)
        | ((uint32_t)messageId);
    }

    [[nodiscard]]
    static constexpr Header fromBits(uint32_t bits) noexcept {
      return {
        .messageType = MessageType(bits >> 26),
        .priority = MessagePriority(bits >> 23),
        .seqNumber = bits >> 20,
        .senderId = bits >> 10,
        .messageId = bits,
      };
    }

    [[nodiscard]]
    static constexpr Header normal(MessageId id) noexcept {
      return {
        .messageType = MessageType::NORMAL,
        .priority = MessagePriority::NORMAL,
        .messageId = id,
      };
    }

    [[nodiscard]]
    static constexpr Header scs(MessageId id) noexcept {
      return {
        .messageId = id,
      };
    }
  };
}
