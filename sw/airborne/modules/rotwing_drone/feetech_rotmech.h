/*
 * Feetech rotating mechanism (wing skew) over CAN/DroneCAN
 * Publishes wing rotation feedback over ABI as ACT_FEEDBACK
 */

#ifndef MODULES_ROTWING_DRONE_FEETECH_ROTMECH_H
#define MODULES_ROTWING_DRONE_FEETECH_ROTMECH_H

#include "std.h"

/* Actuator/servo identifier to accept from Feetech node */
#ifndef FEETECH_ROTMECH_ACTUATOR_ID
#define FEETECH_ROTMECH_ACTUATOR_ID 0
#endif

extern void feetech_rotmech_init(void);
void feetech_rotmech_periodic(void);

/* Instruction helpers */
void feetech_rotmech_cmd_arm(bool arm);
void feetech_rotmech_cmd_target_angle_deg(int16_t angle_deg);
void feetech_rotmech_cmd_serial_freq_hz(uint16_t hz);
void feetech_rotmech_cmd_can_freq_hz(uint16_t hz);
void feetech_rotmech_cmd_log_level(uint8_t long_logs);

/* Exposed settings variables (for DL and settings generator) */
extern float feetech_cfg_can_freq;
// extern float feetech_cfg_serial_freq;
extern uint8_t feetech_cfg_log_level;
// extern uint8_t feetech_cfg_min_angle;
// extern uint8_t feetech_cfg_max_angle;
extern bool feetech_cfg_arm;
extern int16_t feetech_cfg_target_angle;

struct feetech_rotmech_status
{
  uint32_t timestamp;
  uint8_t actuator_id;
  int16_t current_angle;
};

struct feetech_rotmech_debug
{
  uint8_t armed;
  int16_t calculation_offset;
  int16_t target_wing_angle;
  int16_t revolution_count;
  int16_t target_servo_position;
  int16_t current_position_global;
  int16_t current_position;
  int16_t previous_servo_position;
  int16_t current_speed;
  int16_t latest_load;
  uint8_t voltage;
  uint8_t temp;
  uint8_t errcode;
  uint8_t health_state;
};
#endif /* MODULES_ROTWING_DRONE_FEETECH_ROTMECH_H */
