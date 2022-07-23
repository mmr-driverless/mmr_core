#pragma once

namespace mmr::core {
  class Module {
  public:
    void init() = 0;
    void run() = 0;
  };
}
