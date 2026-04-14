/**
 * @file diff_drive_estimator.h
 * @brief Differential drive adapter for the pose estimator interface.
 */

#ifndef DIFF_DRIVE_ESTIMATOR_H
#define DIFF_DRIVE_ESTIMATOR_H

#include "diff_drive.h"
#include "pose_estimator.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialise a pose_estimator_t backed by a diff_drive_t instance.
 *
 * Wires all pose_estimator_t function pointers to the diff_drive
 * implementation. The correct step is a no-op (dead reckoning only).
 *
 * @param estimator  Output: pose_estimator_t to initialise.
 * @param diff_drive Pointer to an already-initialised diff_drive_t.
 */
void diff_drive_estimator_init(pose_estimator_t *estimator,
                               diff_drive_t *diff_drive);

#ifdef __cplusplus
}
#endif

#endif  // DIFF_DRIVE_ESTIMATOR_H
