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

bool dshot_init(uint32_t pin);
void dshot_deinit(void);
bool dshot_set_throttle(uint16_t throttle);
bool dshot_send_command(uint16_t command);
bool dshot_arm(void);
bool dshot_disarm(void);
bool dshot_is_armed(void);

#endif
