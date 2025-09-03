/*
 * Feetech rotating mechanism (wing skew) over CAN/DroneCAN
 * Listens to com.feetech.servo.Status and publishes wing rotation via ABI.
 */

#include "modules/rotwing_drone/feetech_rotmech.h"
#include "uavcan/uavcan.h"
#include "modules/core/abi.h"
#include <math.h>

/* Require Feetech DSDL to be available */
#include "com.feetech.servo.Status.h"
#include "com.feetech.servo.Debug.h"
#include "com.feetech.servo.Config.h"
#include "com.feetech.servo.Instruction.h"


/* Local state */
static uavcan_event feetech_status_ev;
static uavcan_event feetech_debug_ev;
static uavcan_event feetech_config_ev;

struct feetech_rotmech_status feetech_status = {0};
struct feetech_rotmech_debug feetech_debug = {0};
struct rotmech_feetech_state feetech_state = {0};

static struct com_feetech_servo_Status feetech_status_uavcan = {0};
static struct com_feetech_servo_Debug feetech_debug_uavcan = {0};

abi_event wing_skew_cmd_ev;
static void wing_skew_cmd_cb(uint8_t sender_id UNUSED, float angle_deg)
{
  int16_t angle_cdg = (int16_t)(angle_deg * 100.0f);
  feetech_rotmech_cmd_target_angle_deg(angle_cdg);
}

/* ensure we can broadcast (we will broadcast on all enabled interfaces) */

/* Cached settings exposed via DL for one-shot commands */
/* Selectable target actuator id at runtime (defaults to macro) */
uint8_t feetech_target_actuator_id = FEETECH_ROTMECH_ACTUATOR_ID;

/* SuperCAN-like config exposure */
float    feetech_cfg_can_freq = 10.0f;
uint8_t  feetech_cfg_log_level = 0; // 0=short,1=long
// float    feetech_cfg_serial_freq = 100.0f;
// uint8_t  feetech_cfg_min_angle = 0;
// uint8_t  feetech_cfg_max_angle = 90;

bool     feetech_cfg_arm = true;


static void feetech_status_cb(struct uavcan_iface_t *iface __attribute__((unused)), CanardRxTransfer *transfer)
{
  if (com_feetech_servo_Status_decode(transfer, &feetech_status_uavcan)) {
    return; // decode error
  }

  if (feetech_status_uavcan.actuator_id != feetech_target_actuator_id) {
    return; // not our servo
  }

  // Copy status
  feetech_status.timestamp = get_sys_time_usec();
  feetech_status.actuator_id =  feetech_status_uavcan.actuator_id;
  feetech_status.current_angle =  feetech_status_uavcan.current_angle;
  feetech_debug.debug_enabled = 0;

  feetech_state.status = feetech_status;
  feetech_state.debug = feetech_debug;

  AbiSendMsgWING_SKEW_STATE(ABI_BROADCAST, &feetech_state);
}

static void feetech_debug_cb(struct uavcan_iface_t *iface __attribute__((unused)), CanardRxTransfer *transfer)
{
  if (com_feetech_servo_Debug_decode(transfer, &feetech_debug_uavcan)) {
    return; // decode error
  }

  if (feetech_debug_uavcan.actuator_id != feetech_target_actuator_id) {
    return; // not our servo
  }

  // Copy status
  feetech_status.timestamp = get_sys_time_usec();
  feetech_status.actuator_id =  feetech_debug_uavcan.actuator_id;
  feetech_status.current_angle =  feetech_debug_uavcan.current_angle;

  feetech_debug.debug_enabled = 1;
  feetech_debug.armed = feetech_debug_uavcan.armed;
  feetech_debug.calculation_offset = feetech_debug_uavcan.calculation_offset;
  feetech_debug.target_wing_angle = feetech_debug_uavcan.target_wing_angle;
  feetech_debug.revolution_count = feetech_debug_uavcan.revolution_count;
  feetech_debug.target_servo_position = feetech_debug_uavcan.target_servo_position;
  feetech_debug.current_position_global = feetech_debug_uavcan.current_position_global;
  feetech_debug.current_position = feetech_debug_uavcan.current_position;
  feetech_debug.previous_servo_position = feetech_debug_uavcan.previous_servo_position;
  feetech_debug.current_speed = feetech_debug_uavcan.current_speed;
  feetech_debug.latest_load = feetech_debug_uavcan.latest_load;
  feetech_debug.voltage = feetech_debug_uavcan.voltage;
  feetech_debug.temp = feetech_debug_uavcan.temp;
  feetech_debug.errcode = feetech_debug_uavcan.errcode;
  feetech_debug.health_state = feetech_debug_uavcan.health_state;

  feetech_state.status = feetech_status;
  feetech_state.debug = feetech_debug;

  AbiSendMsgWING_SKEW_STATE(ABI_BROADCAST, &feetech_state);
}

static void feetech_config_cb(struct uavcan_iface_t *iface __attribute__((unused)), CanardRxTransfer *transfer)
{
  struct com_feetech_servo_Config msg;
  if (com_feetech_servo_Config_decode(transfer, &msg)) {
    return; // decode error
  }
  if (msg.actuator_id != feetech_target_actuator_id) {
    return;
  }
  // Could export to telemetry or compare desired vs actual frequencies if needed.
}

static inline void feetech_broadcast_instruction(const struct com_feetech_servo_Instruction *msg)
{
#if UAVCAN_USE_CAN1
  extern struct uavcan_iface_t uavcan1;
  uavcan_broadcast(&uavcan1, COM_FEETECH_SERVO_INSTRUCTION_SIGNATURE, COM_FEETECH_SERVO_INSTRUCTION_ID,
                   CANARD_TRANSFER_PRIORITY_MEDIUM, msg, sizeof(*msg));
#endif
#if UAVCAN_USE_CAN2
  extern struct uavcan_iface_t uavcan2;
  uavcan_broadcast(&uavcan2, COM_FEETECH_SERVO_INSTRUCTION_SIGNATURE, COM_FEETECH_SERVO_INSTRUCTION_ID,
                   CANARD_TRANSFER_PRIORITY_MEDIUM, msg, sizeof(*msg));
#endif
}

static inline void feetech_send_instruction(uint8_t msg_type, uint8_t d0, uint8_t d1)
{
  struct com_feetech_servo_Instruction ins = {0};
  ins.actuator_id = feetech_target_actuator_id;
  ins.message_type = msg_type;
  ins.data[0] = d0;
  ins.data[1] = d1;
  feetech_broadcast_instruction(&ins);
}

/* Public API */
void feetech_rotmech_cmd_arm(bool arm)
{
  feetech_cfg_arm = arm;
  feetech_send_instruction(0, arm ? 1 : 0, 0); // ARM_DISARM
}

void feetech_rotmech_cmd_target_angle_deg(int16_t angle_deg)
{
  feetech_send_instruction(1, angle_deg & 0xFF, (angle_deg >> 8) & 0xFF); // SET_TARGET_ANGLE
}

// void feetech_rotmech_cmd_serial_freq_hz(uint16_t hz)
// {
//   feetech_cfg_serial_freq = hz;
//   feetech_send_instruction(2, (uint8_t)(hz & 0xFF), (uint8_t)((hz >> 8) & 0xFF)); // SET_SERIAL_FREQ
// }

void feetech_rotmech_cmd_can_freq_hz(uint16_t hz)
{
  feetech_cfg_can_freq = hz;
  feetech_send_instruction(3, (uint8_t)(hz & 0xFF), (uint8_t)((hz >> 8) & 0xFF)); // SET_CAN_FREQ
}

void feetech_rotmech_cmd_log_level(uint8_t long_logs)
{
  feetech_cfg_log_level = long_logs;
  feetech_send_instruction(4, long_logs ? 1 : 0, 0); // SET_LOG_LEVEL
}

void feetech_rotmech_init(void)
{
  // Bind to Feetech Status message
  uavcan_bind(COM_FEETECH_SERVO_STATUS_ID, COM_FEETECH_SERVO_STATUS_SIGNATURE,
              &feetech_status_ev, &feetech_status_cb);

  // Bind to Feetech Debug message
  uavcan_bind(COM_FEETECH_SERVO_DEBUG_ID, COM_FEETECH_SERVO_DEBUG_SIGNATURE,
              &feetech_debug_ev, &feetech_debug_cb);

  // Bind config feedback (optional)
  uavcan_bind(COM_FEETECH_SERVO_CONFIG_ID, COM_FEETECH_SERVO_CONFIG_SIGNATURE,
              &feetech_config_ev, &feetech_config_cb);


  AbiBindMsgWING_SKEW_CMD(ABI_BROADCAST, &wing_skew_cmd_ev, wing_skew_cmd_cb);
}
