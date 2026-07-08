#ifndef DSHOT_H
#define DSHOT_H

#include <stdbool.h>
#include <stdint.h>

#define DSHOT_MIN_THROTTLE 48u
#define DSHOT_MAX_THROTTLE 2047u
#define DSHOT_CMD_MOTOR_STOP 0u
#define DSHOT_CMD_BEEP1 1u
#define DSHOT_CMD_BEEP2 2u
#define DSHOT_CMD_BEEP3 3u
#define DSHOT_CMD_BEEP4 4u
#define DSHOT_CMD_BEEP5 5u
#define DSHOT_CMD_ESC_INFO 6u
#define DSHOT_CMD_SAVE_SETTINGS 12u
#define DSHOT_CMD_RESET 13u

// Initialize DShot on 4 consecutive pins starting at `base_pin`.
// Returns true if all PIO state machines were successfully configured.
bool dshot_init(uint32_t base_pin);
void dshot_deinit(void);

// Set throttle for a specific motor index [0..3].
bool dshot_set_throttle(uint8_t motor, uint16_t throttle);

// Send a raw DShot packet payload to a single motor.
// The payload is the 11-bit value that will be encoded in the frame.
bool dshot_send_raw_packet(uint8_t motor, uint16_t packet_value, bool telemetry);

// Send a DShot command to all motors (telemetry bit set).
bool dshot_send_command(uint16_t command);
bool dshot_arm(void);
bool dshot_disarm(void);
bool dshot_is_armed(void);

#endif
