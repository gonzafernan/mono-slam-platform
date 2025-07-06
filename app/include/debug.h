/**
 * @file debug.h
 * @brief Header file for debug interface module.
 */

#ifndef DEBUG_H
#define DEBUG_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Debug interface initialization
 * @param context Pointer to the transport context
 */
void debug_init(void *context);

#ifdef __cplusplus
}
#endif

#endif  // DEBUG_H