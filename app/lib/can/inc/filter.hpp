#pragma once
#include <cstdint>

namespace mmr::can {
  struct Filter {
    const uint32_t id;
    const uint32_t mask;
    const bool isExtendedId;
  };
}
