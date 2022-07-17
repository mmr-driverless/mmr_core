#pragma once
#include "message.hpp"
#include "header.hpp"
#include "filter.hpp"

#include <task.hpp>
#include <cstdint>
#include <literals.hpp>
#include <string_view>

using namespace mmr::util;

namespace mmr::can {
  class CanBus {
  public:
    CanBus() = delete;

    [[nodiscard]] bool setFilter(Filter filter);
    [[nodiscard]] bool send(Message message);

    [[nodiscard]] uint8_t getPendingMessages();
    [[nodiscard]] Message receive();
    [[nodiscard]] Task<Message> receiveAsync();

  protected:
    virtual bool trySend(Message& message) = 0;
    virtual bool tryReceive(Message& message) = 0;
    virtual bool trySetFilter(Filter& filter) = 0;
    virtual uint8_t getPendingMessages() = 0;
  };
}
