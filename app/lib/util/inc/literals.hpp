#pragma once

#include <cstdint>

namespace mmr::util {
  constexpr uint64_t operator ""_Hz(uint64_t freq) {
    return freq;
  }

  constexpr uint64_t operator ""_MHz(uint64_t freq) {
    return freq * 1000;
  }

  constexpr uint64_t operator ""_GHz(uint64_t freq) {
    return freq * 1000_MHz;
  }
}
