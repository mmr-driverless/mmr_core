/**
 * @file mmr_can_optimize.h
 * @brief
 * Low level optimization utilities.
 */

#ifndef INC_MMR_CAN_OPTIMIZE_H_
#define INC_MMR_CAN_OPTIMIZE_H_

#ifdef __GNUC__
/**
 * @brief
 * Tells the compiler that the given method
 * must always be inlined.
 *
 * @example
 * static always_inline int min(int a, int b) {
 *   return a < b ? a : b;
 * }
 */
#define always_inline inline __attribute__((always_inline))
#else
#define always_inline inline
#endif

#endif /* INC_MMR_CAN_OPTIMIZE_H_ */
