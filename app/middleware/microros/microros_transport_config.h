/**
 * @file microros_transport_config.h
 * @brief Configuration for micro-ROS transport layer
 */

#ifndef MICROROS_TRANSPORT_CONFIG_H
#define MICROROS_TRANSPORT_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <rcl/rcl.h>

typedef struct {
    rcl_node_t *node;
} transport_context_t;

#ifdef __cplusplus
}
#endif
#endif  // MICROROS_TRANSPORT_CONFIG_H