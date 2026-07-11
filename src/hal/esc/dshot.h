#ifndef DSHOT_H
#define DSHOT_H

#include <stdbool.h>
#include <stdint.h>

#define DSHOT_MIN_THROTTLE 70u
#define DSHOT_MAX_THROTTLE 2047u

// Initialize DShot on 4 consecutive pins starting at `base_pin`.
// Returns true if all PIO state machines were successfully configured.
bool dshot_init(uint32_t base_pin);

// Set throttle for a specific motor index [0..3].
bool dshot_set_throttle(uint8_t motor, uint16_t throttle);

bool dshot_arm(void);
bool dshot_disarm(void);

#endif
