
typedef enum Error {
  TIMEOUT,
  CAN_ERROR,
};

struct CanError {
  int interface;
};

struct ErrorResult {
  Errors errorCode;
  union {
    CanError can;
  } info;
};