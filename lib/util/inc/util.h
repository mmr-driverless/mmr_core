/**
 * @file mmr_can_util.h
 * @brief
 * Utility functions and macros.
 */

#ifndef INC_MMR_UTIL_H_
#define INC_MMR_UTIL_H_

#include <stdint.h>
#include <string.h>

/**
 * @brief
 * Returns the size of a given array.
 *
 * This only works on static arrays declared
 * within the current scope, that is:
 * int main() {
 *   int arr[] = {1, 2, 3};
 *   int len = arraylen(arr);
 * }
 */
#define arraylen(array) \
  (sizeof(array) / sizeof(*(array)))

/**
 * @brief
 * Returns the minimum between the
 * two given values.
 * E.g. min(1, 2) == 1 // true
 */
#define min(a, b) ((a) < (b) ? a : b)
#define mask(value, bits) (value & bits)
#define allLow(bits) (bits == 0)
#define allHigh(bits) (bits == ~0)

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
typedef enum MmrTaskResult {
  MMR_TASK_ERROR,
  MMR_TASK_PENDING,
  MMR_TASK_COMPLETED,
} MmrTaskResult;


typedef uint8_t MmrBitVector8b;

#endif
