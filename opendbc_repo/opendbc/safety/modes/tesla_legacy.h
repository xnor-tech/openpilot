#pragma once

#include "opendbc/safety/safety_declarations.h"

static bool tesla_external_panda = false;
static bool tesla_hw1 = false;
static bool tesla_hw2 = false;
static bool tesla_hw3 = false;

static int chassis_bus = 0U;
static int das_control_msg = 0x2bfU;
static int di_torque1_msg = 0x106U;

static bool tesla_legacy_stock_aeb = false;

// Only rising edges while controls are not allowed are considered for these systems:
// TODO: Only LKAS (non-emergency) is currently supported since we've only seen it
static bool tesla_legacy_stock_lkas = false;
static bool tesla_legacy_stock_lkas_prev = false;

static void tesla_legacy_rx_hook(const CANPacket_t *msg) {

  // Steering angle: (0.1 * val) - 819.2 in deg.
  if (!tesla_external_panda && (msg->bus == 0U) && (msg->addr == 0x370U)) {
    // Store it 1/10 deg to match steering request
    const int angle_meas_new = (((msg->data[4] & 0x3FU) << 8) | msg->data[5]) - 8192U;
    update_sample(&angle_meas, angle_meas_new);

    const int hands_on_level = msg->data[4] >> 6;  // handsOnLevel
    const int eac_status = msg->data[6] >> 5;      // eacStatus
    const int eac_error_code = msg->data[2] >> 4;  // eacErrorCode

    // Disengage on normal user override, or if high angle rate fault from user overriding extremely quickly
    steering_disengage = (hands_on_level >= 3) || ((eac_status == 0) && (eac_error_code == 9));
  }

  // Vehicle speed (ESP_B: ESP_vehicleSpeed)
  if ((!tesla_external_panda) && (msg->bus == chassis_bus) && (msg->addr == 0x155U)) {
    // Vehicle speed: (0.01 * val) * KPH_TO_MS
    float speed = ((msg->data[6] | (msg->data[5] << 8)) * 0.01) * KPH_TO_MS;
    UPDATE_VEHICLE_SPEED(speed);
  }

  // Gas pressed
  if ((tesla_external_panda || tesla_hw1) && (msg->bus == 0U) && (msg->addr == di_torque1_msg)) {
    gas_pressed = msg->data[6] != 0U;
  }

  if (((tesla_external_panda) && (msg->bus == 0U) && (msg->addr == 0x1f8U)) ||
     ((!tesla_external_panda) && (msg->bus == chassis_bus) && (msg->addr == 0x20aU))) {
    brake_pressed = (((msg->data[0] & 0x0CU) >> 2) != 1U);
  }

  // Cruise
  if (((tesla_external_panda) && (msg->bus == 0U) && (msg->addr == 0x256U)) ||
     ((!tesla_external_panda) && (msg->bus == chassis_bus) && (msg->addr == 0x368U))) {
      // Cruise state
      int cruise_state = (msg->data[1] >> 4) & 0x07U;
      bool cruise_engaged = (cruise_state == 2) ||  // ENABLED
                            (cruise_state == 3) ||  // STANDSTILL
                            (cruise_state == 4) ||  // OVERRIDE
                            (cruise_state == 6) ||  // PRE_FAULT
                            (cruise_state == 7);    // PRE_CANCEL
      vehicle_moving = cruise_state != 3; // STANDSTILL
      pcm_cruise_check(cruise_engaged);
   }

  if (msg->bus == 2U) {
    // DAS_control
    if ((tesla_external_panda || tesla_hw1) && msg->addr == das_control_msg) {
      // "AEB_ACTIVE"
      tesla_legacy_stock_aeb = (msg->data[2] & 0x03U) == 1U;
    }

    // DAS_steeringControl
    if (!tesla_external_panda && msg->addr == 0x488U) {
      int steering_control_type = msg->data[2] >> 6;
      bool tesla_legacy_stock_lkas_now = steering_control_type == 2;  // "LANE_KEEP_ASSIST"

      // Only consider rising edges while controls are not allowed
      if (tesla_legacy_stock_lkas_now && !tesla_legacy_stock_lkas_prev && !controls_allowed) {
        tesla_legacy_stock_lkas = true;
      }
      if (!tesla_legacy_stock_lkas_now) {
        tesla_legacy_stock_lkas = false;
      }
      tesla_legacy_stock_lkas_prev = tesla_legacy_stock_lkas_now;
    }
  }
}


static bool tesla_legacy_tx_hook(const CANPacket_t *msg) {
  const AngleSteeringLimits TESLA_STEERING_LIMITS = {
    .max_angle = 3600,  // 360 deg, EPAS faults above this
    .angle_deg_to_can = 10,
    .frequency = 50U,
  };

  // NOTE: based off TESLA_MODEL_S_HW3 to match openpilot
  const AngleSteeringParams TESLA_LEGACY_STEERING_PARAMS = {
    .slip_factor = -0.0005666493436310427,  // calc_slip_factor(VM)
    .steer_ratio = 15.,
    .wheelbase = 2.96,
  };

  const LongitudinalLimits TESLA_LONG_LIMITS = {
    .max_accel = 425,       // 2 m/s^2
    .min_accel = 288,       // -3.48 m/s^2
    .inactive_accel = 375,  // 0. m/s^2
  };

  bool tx = true;
  bool violation = false;

  // Steering control: (0.1 * val) - 1638.35 in deg.
  if (!tesla_external_panda && (msg->addr == 0x488U)) {
    // We use 1/10 deg as a unit here
    int raw_angle_can = ((msg->data[0] & 0x7FU) << 8) | msg->data[1];
    int desired_angle = raw_angle_can - 16384;
    int steer_control_type = msg->data[2] >> 6;
    bool steer_control_enabled = steer_control_type == 1;  // ANGLE_CONTROL

    if (steer_angle_cmd_checks_vm(desired_angle, steer_control_enabled, TESLA_STEERING_LIMITS, TESLA_LEGACY_STEERING_PARAMS)) {
      violation = true;
    }

    bool valid_steer_control_type = (steer_control_type == 0) ||  // NONE
                                    (steer_control_type == 1);    // ANGLE_CONTROL
    if (!valid_steer_control_type) {
      violation = true;
    }

    if (tesla_legacy_stock_lkas) {
      // Don't allow any steering commands when stock LKAS is active
      violation = true;
    }
  }

  // DAS_control: longitudinal control message
  if ((tesla_external_panda || tesla_hw1) && (msg->addr == das_control_msg)) {
    // No AEB events may be sent by openpilot
    int aeb_event = msg->data[2] & 0x03U;
    if (aeb_event != 0) {
      violation = true;
    }

    // Don't send long/cancel messages when the stock AEB system is active
    if (tesla_legacy_stock_aeb) {
      violation = true;
    }

    int raw_accel_max = ((msg->data[6] & 0x1FU) << 4) | (msg->data[5] >> 4);
    int raw_accel_min = ((msg->data[5] & 0x0FU) << 5) | (msg->data[4] >> 3);

    // Prevent both acceleration from being negative, as this could cause the car to reverse after coming to standstill
    if ((raw_accel_max < TESLA_LONG_LIMITS.inactive_accel) && (raw_accel_min < TESLA_LONG_LIMITS.inactive_accel)) {
      violation = true;
    }

    // Don't allow any acceleration limits above the safety limits
    violation |= longitudinal_accel_checks(raw_accel_max, TESLA_LONG_LIMITS);
    violation |= longitudinal_accel_checks(raw_accel_min, TESLA_LONG_LIMITS);
  }

  if (violation) {
    tx = false;
  }

  return tx;
}

static bool tesla_legacy_fwd_hook(int bus_num, int addr) {
  bool block_msg = false;

  if (bus_num == 2) {
    // APS_eacMonitor
    if (!tesla_external_panda && !tesla_hw1 && (addr == 0x27dU)) {
      block_msg = true;
    }

    // DAS_steeringControl
    if (!tesla_external_panda && (addr == 0x488U) && !tesla_legacy_stock_lkas) {
      block_msg = true;
    }

    // DAS_control
    if ((tesla_external_panda || tesla_hw1) && (addr == das_control_msg) && !tesla_legacy_stock_aeb) {
      block_msg = true;
    }
  }

  return block_msg;
}

static safety_config tesla_legacy_init(uint16_t param) {
  const int TESLA_FLAG_EXTERNAL_PANDA = 2;
  const int TESLA_FLAG_HW1 = 4;
  const int TESLA_FLAG_HW2 = 8;
  const int TESLA_FLAG_HW3 = 16;

  // Extract flags
  tesla_external_panda = GET_FLAG(param, TESLA_FLAG_EXTERNAL_PANDA);
  tesla_hw1 = GET_FLAG(param, TESLA_FLAG_HW1);
  tesla_hw2 = GET_FLAG(param, TESLA_FLAG_HW2);
  tesla_hw3 = GET_FLAG(param, TESLA_FLAG_HW3);

  // Initialize state variables
  tesla_legacy_stock_aeb = false;
  tesla_legacy_stock_lkas = false;
  tesla_legacy_stock_lkas_prev = false;
  chassis_bus = 0U;
  di_torque1_msg = 0x106U;

  // Set DAS control message address
  das_control_msg = tesla_external_panda ? 0x2bfU : 0x2b9U;

  // Define message arrays (keeping them as is)
  static const CanMsg TESLA_TX_LEGACY_MSGS[] = {
    {0x488, 0, 4, .check_relay = true, .disable_static_blocking = true},  // DAS_steeringControl
    {0x27D, 0, 3, .check_relay = true, .disable_static_blocking = true},  // APS_eacMonitor
  };

  static const CanMsg TESLA_LEGACY_PT_MSGS[] = {
    {0x2bf, 0, 8, .check_relay = true, .disable_static_blocking = true},  // DAS_control
  };

  static const CanMsg TESLA_TX_LEGACY_HW1_MSGS[] = {
    {0x488, 0, 4, .check_relay = true, .disable_static_blocking = true},  // DAS_steeringControl
    {0x2b9, 0, 8, .check_relay = true, .disable_static_blocking = true},  // DAS_control
  };

  // Define RX check arrays (keeping them as is)
  static RxCheck tesla_legacy_pt_rx_checks[] = {
    {.msg = {{0x106, 0, 8, 100U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},  // DI_torque1
    {.msg = {{0x1f8, 0, 8, 50U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},   // BrakeMessage
    {.msg = {{0x2bf, 2, 8, 25U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},   // DAS_control
    {.msg = {{0x256, 0, 8, 10U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},   // DI_state
  };

  static RxCheck tesla_legacy_hw1_rx_checks[] = {
    {.msg = {{0x108, 0, 8, 100U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},  // DI_torque1
    {.msg = {{0x2b9, 2, 8, 25U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},   // DAS_control
    {.msg = {{0x370, 0, 8, 25U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},   // EPAS_sysStatus (25hz)
    {.msg = {{0x155, 0, 8, 50U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},   // ESP_private1
    {.msg = {{0x20a, 0, 8, 50U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},   // BrakeMessage
    {.msg = {{0x368, 0, 8, 10U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},   // DI_state
    {.msg = {{0x488, 2, 4, 50U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},   // DAS_steeringControl
  };

  static RxCheck tesla_legacy_hw2_rx_checks[] = {
    {.msg = {{0x370, 0, 8, 25U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},   // EPAS_sysStatus (25hz)
    {.msg = {{0x155, 0, 8, 50U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},   // ESP_private1
    {.msg = {{0x20a, 0, 8, 50U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},   // BrakeMessage
    {.msg = {{0x368, 0, 8, 10U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},   // DI_state
    {.msg = {{0x488, 2, 4, 50U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},   // DAS_steeringControl
  };

  static RxCheck tesla_legacy_hw3_rx_checks[] = {
    {.msg = {{0x370, 0, 8, 100U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},   // EPAS_sysStatus (100hz)
    {.msg = {{0x155, 1, 8, 50U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},   // ESP_private1
    {.msg = {{0x20a, 1, 8, 50U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},   // BrakeMessage
    {.msg = {{0x368, 1, 8, 10U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},   // DI_state
    {.msg = {{0x488, 2, 4, 50U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},   // DAS_steeringControl
  };

  // Determine configuration based on hardware type
  if (tesla_external_panda && (tesla_hw3 || tesla_hw2)) {
    return BUILD_SAFETY_CFG(tesla_legacy_pt_rx_checks, TESLA_LEGACY_PT_MSGS);
  }

  if (tesla_hw3) {
    chassis_bus = 1U;
    return BUILD_SAFETY_CFG(tesla_legacy_hw3_rx_checks, TESLA_TX_LEGACY_MSGS);
  }

  if (tesla_hw1) {
    di_torque1_msg = 0x108U;
    return BUILD_SAFETY_CFG(tesla_legacy_hw1_rx_checks, TESLA_TX_LEGACY_HW1_MSGS);
  }

  // Default case: HW2
  return BUILD_SAFETY_CFG(tesla_legacy_hw2_rx_checks, TESLA_TX_LEGACY_MSGS);
}

const safety_hooks tesla_legacy_hooks = {
  .init = tesla_legacy_init,
  .rx = tesla_legacy_rx_hook,
  .tx = tesla_legacy_tx_hook,
  .fwd = tesla_legacy_fwd_hook,
};
