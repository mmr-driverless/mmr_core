#pragma once

namespace mmr::util {
  enum class TaskStatus {
    COMPLETED,
    PENDING,
    ERROR,
  };

  struct Unit {};

  template <typename TResult>
  struct Task {
    TaskStatus status;
    TResult result;
  };
}
