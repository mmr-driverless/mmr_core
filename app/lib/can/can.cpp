#include "inc/can.hpp"
#include <util.h>

using namespace mmr::can;


[[nodiscard]]
bool CanBus::setFilter(Filter filter) {
  return trySetFilter(filter);
}

[[nodiscard]]
bool CanBus::send(Message message) {
  return trySend(message);
}

[[nodiscard]]
Message CanBus::receive() {
  auto result = Message{};
  tryReceive(result);

  return result;
}

[[nodiscard]]
uint8_t CanBus::getPendingMessages() {
  return getPendingMessages();
}

