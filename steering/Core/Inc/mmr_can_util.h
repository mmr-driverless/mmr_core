/**
 * @file mmr_can_util.h
 * @brief
 * Utility functions and macros.
 */

#ifndef INC_MMR_CAN_UTIL_H_
#define INC_MMR_CAN_UTIL_H_

#include <stdint.h>
#include <string.h>

/**
 * @brief
 * Returns the size of the given array.
 *
 * This only works on static arrays declared
 * within the current scope, that is:
 * int main() {
 *   int arr[] = {1, 2, 3};
 *   int len = sizeofarray(arr);
 * }
 */
#define sizeofarray(array) \
  (sizeof(array) / sizeof(*(array)))

/**
 * @brief
 * Returns the length of a statically, non const ptr, declared
 * string.
 * That is:
 * int main() {
 *   char str[] = "abc";
 *   int len = stringArrayLength(str);
 * }
 */
#define stringArrayLength(array) \
  stringBufferLength((array), sizeofarray(array))

/**
 * @brief
 * Returns the length of a string buffer.
 * Its format must be in bytes, so it could be:
 *   const char*
 *   char*
 *   uint8_t*
 *   etc...
 */
#define stringBufferLength(pbuffer, maxLen) \
  strnlen((const char*)(pbuffer), maxLen)

/**
 * @brief
 * Returns the minimum between the
 * two given values.
 * E.g. min(1, 2) == 1 // true
 */
#define min(a, b) ((a) < (b) ? a : b)
#define mask(value, bits) (value & bits)
#define convertTo(resultType, lvalue) (*interpretAs(resultType*, &(lvalue)))
#define interpretAs(resultType, lvalue) ((resultType)(lvalue))


/**
 * @brief
 * Represents the result of an asynchronous computation
 *
 * Either
 *  - Error: the computation resulted in error
 *  - Pending: the computation is still undergoing
 *  - Completed: the computation has completed successfully
 *               and its results can be read
 *
 * Asynchronous logic can be easily implemented using State Machines
 */
typedef enum MmrResult {
  MMR_RESULT_ERROR,
  MMR_RESULT_PENDING,
  MMR_RESULT_COMPLETED,
} MmrResult;


#endif /* INC_MMR_CAN_UTIL_H_ */
