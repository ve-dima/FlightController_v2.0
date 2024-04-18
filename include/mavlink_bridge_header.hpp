#pragma once
/* use efficient approach, see mavlink_helpers.h */
#define MAVLINK_SEND_UART_BYTES mavlink_send_uart_bytes

#include "mavlink/mavlink_types.h"
extern mavlink_system_t mavlink_system;
extern void mavlink_send_uart_bytes(mavlink_channel_t chan, const uint8_t *ch, uint16_t length);
