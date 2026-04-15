// opendbc_repo/opendbc/safety/modes/tesla_legacy.h
#pragma once

#include "opendbc/safety/declarations.h"

// Tesla Legacy (HW1/HW2/HW3) Unity-parity safety for XNOR harnessing.
//
// Unity parity implemented:
//  - Stalk (0x45) gated by op_autopilot_disabled from internal 0x659 (byte5 bit7)
//  - Forwarding mods (HUD hiding + EPAS eacStatus workaround) with additive last-byte checksum
//  - No relay faults: all TX allowlist entries use check_relay=false
//
// XNOR contract:
//  - safetyParam uses TeslaSafetyFlags (opendbc_repo/opendbc/car/tesla/values.py):
//      LONG_CONTROL=1, FSD_14=2, FLAG_EXTERNAL_PANDA=4, FLAG_HW1=8, FLAG_HW2=16, FLAG_HW3=32, OP_STALK_ENABLE=64
//  - Main panda: lateral TX + stock LKAS passthrough + Unity forwarding mods
//  - External panda: longitudinal TX + stock AEB passthrough

// --- safetyParam bits (TeslaSafetyFlags) ---
#define TESLA_LEGACY_FLAG_LONG_CONTROL       0x01U
#define TESLA_LEGACY_FLAG_FSD_14             0x02U
#define TESLA_LEGACY_FLAG_EXTERNAL_PANDA     0x04U
#define TESLA_LEGACY_FLAG_HW1                0x08U
#define TESLA_LEGACY_FLAG_HW2                0x10U
#define TESLA_LEGACY_FLAG_HW3                0x20U
#define TESLA_LEGACY_FLAG_OP_STALK_ENABLE    0x40U

// --- Unity timing ---
static const uint32_t TESLA_LEGACY_TIME_TO_HIDE_ERRORS_US = 4000000U;
static const uint32_t TESLA_LEGACY_TIME_FOR_HANDS_ON_US   = 1000000U;

// --- runtime state (namespaced; safety.h includes tesla.h too) ---
static bool tesla_legacy_external_panda = false;
static bool tesla_legacy_has_ap_hw = false;
static bool tesla_legacy_op_stalk_enable = false;
static int tesla_legacy_chassis_bus = 0;

// internal OP->safety carrier (0x659) — Unity parity bits (byte5)
static bool tesla_legacy_op_autopilot_disabled = false;  // bit7
static bool tesla_legacy_pedal_enabled = false;          // 0x659 byte5 bit5 (Unity parity)
static bool tesla_legacy_op_stalk_main_edge = false;     // bit1 (edge)
static bool tesla_legacy_op_stalk_cancel_edge = false;   // bit0 (edge)

// stock system detection on AP-side bus (bus2)
static bool tesla_legacy_stock_lkas = false;  // from 0x488 steerControlType
static bool tesla_legacy_stock_aeb = false;   // from 0x2BF AEB event

// stock AP states from DAS/AP frames
static bool tesla_legacy_autopilot_enabled = false;  // 0x399
static bool tesla_legacy_eac_enabled = false;        // 0x219
static bool tesla_legacy_autopark_enabled = false;   // 0x219

// hands on wheel (from 0x370)
static bool tesla_legacy_hands_on = false;
static uint32_t tesla_legacy_hands_on_last_signal = 0U;

// time tracking for HUD hiding after disengage
static uint32_t tesla_legacy_time_op_disengaged = 0U;
static bool tesla_legacy_controls_allowed_prev = false;
static bool tesla_legacy_hide_errors_armed = false;

// gear tracking for reverse -> drive re-arm
static bool tesla_legacy_in_reverse = false;
static uint8_t tesla_legacy_last_gear = 0U;

// local safety reset used on reverse entry and reverse -> drive
static void tesla_legacy_reset_after_gear_change(void) {
  controls_allowed = false;
  cruise_engaged_prev = false;
  steering_disengage = false;

  tesla_legacy_stock_lkas = false;
  tesla_legacy_stock_aeb = false;
  tesla_legacy_autopilot_enabled = false;
  tesla_legacy_eac_enabled = false;
  tesla_legacy_autopark_enabled = false;

  tesla_legacy_op_stalk_main_edge = false;
  tesla_legacy_op_stalk_cancel_edge = false;

  tesla_legacy_time_op_disengaged = 0U;
  tesla_legacy_controls_allowed_prev = false;
  tesla_legacy_hide_errors_armed = false;
}


// --- helpers ---

static void tesla_legacy_track_controls_allowed_edge(void) {
  if (tesla_legacy_controls_allowed_prev && !controls_allowed) {
    tesla_legacy_time_op_disengaged = microsecond_timer_get();
    tesla_legacy_hide_errors_armed = true;
    tesla_legacy_controls_allowed_prev = false;
  } else if (controls_allowed) {
    tesla_legacy_hide_errors_armed = false;
  }
  tesla_legacy_controls_allowed_prev = controls_allowed;
}

static uint8_t tesla_legacy_calc_checksum8(const CANPacket_t *msg, int len) {
  // Unity parity: additive checksum includes addr low+high bytes.
  // Special-case: 0x2BF uses 0x2B9 for checksum.
  uint16_t addr = (uint16_t)msg->addr;
  if (addr == 0x2BFU) {
    addr = 0x2B9U;
  }
  uint8_t checksum = (uint8_t)(addr & 0xFFU) + (uint8_t)((addr >> 8) & 0xFFU);
  for (int i = 0; i < (len - 1); i++) {
    checksum = (uint8_t)(checksum + msg->data[i]);
  }
  return checksum;
}

static void tesla_legacy_set_last_byte_checksum(CANPacket_t *msg) {
  const int len = GET_LEN(msg);
  if (len > 0) {
    msg->data[len - 1] = tesla_legacy_calc_checksum8(msg, len);
  }
}

// --- RX hook ---
static void tesla_legacy_rx_hook(const CANPacket_t *msg) {
  const int bus = (int)msg->bus;
  const int addr = (int)msg->addr;

  // Chassis state (HW3 uses bus1)
  if (bus == tesla_legacy_chassis_bus) {
    if (addr == 0x108) {  // DI_torque1
      const float pedal_pct = ((float)msg->data[6]) * 0.4f;  // DI_pedalPos
      gas_pressed = pedal_pct > 3.0f;
    } else if (addr == 0x20A) {  // BrakeMessage
      const uint8_t st = (msg->data[0] >> 2) & 0x3U;         // driverBrakeStatus
      brake_pressed = (st == 2U);
    } else if (addr == 0x368) {  // DI_state
      const uint8_t cruise_state = (msg->data[1] >> 4) & 0x0FU;
      const bool cruise_engaged = (cruise_state == 2U) || (cruise_state == 3U);
      vehicle_moving = (cruise_state != 3U);

      if (!tesla_legacy_op_autopilot_disabled) {
        pcm_cruise_check(cruise_engaged);
      }
    } else if (addr == 0x155) {  // ESP_B (vehicle speed)
      const uint16_t raw_kph = (uint16_t)((((uint16_t)msg->data[5]) << 8) | msg->data[4]);
      const float speed_ms = ((float)raw_kph) * 0.01f * (float)KPH_TO_MS;
      UPDATE_VEHICLE_SPEED(speed_ms);
    } else {
    }
  }

  // Gear state (DI_torque2 0x280) on chassis / mirrored PT bus
  if (((bus == tesla_legacy_chassis_bus) || (bus == 2)) && (addr == 0x280)) {
    const uint8_t gear = (msg->data[1] >> 4) & 0x07U;
    const bool prev_reverse = tesla_legacy_in_reverse;
    const bool now_reverse = gear == 2U;  // DI_GEAR_R
    const bool now_drive = gear == 4U;    // DI_GEAR_D

    if (now_reverse && !prev_reverse) {
      tesla_legacy_reset_after_gear_change();
    } else if (prev_reverse && now_drive) {
      tesla_legacy_reset_after_gear_change();
    }

    tesla_legacy_in_reverse = now_reverse;
    tesla_legacy_last_gear = gear;
  }

  // EPAS_sysStatus (0x370) on bus0
  if ((bus == 0) && (addr == 0x370)) {
    const int angle_meas_new = (((msg->data[4] & 0x3FU) << 8) | msg->data[5]) - 8192;
    update_sample(&angle_meas, angle_meas_new);

    const int hands_on_level = (msg->data[4] >> 6) & 0x03;
    const int eac_status = (msg->data[6] >> 5) & 0x07;
    const int eac_error_code = (msg->data[2] >> 4) & 0x0F;

    tesla_legacy_hands_on = hands_on_level > 0;
    if (tesla_legacy_hands_on) {
      tesla_legacy_hands_on_last_signal = microsecond_timer_get();
    } else {
      const uint32_t dt = get_ts_elapsed(microsecond_timer_get(), tesla_legacy_hands_on_last_signal);
      tesla_legacy_hands_on = dt <= TESLA_LEGACY_TIME_FOR_HANDS_ON_US;
    }

    // Unity parity: do NOT disengage on hands-on escalation; only on EPAS inhibit/error.
    const bool disengage = ((eac_status == 0) && (eac_error_code == 9));
    steering_disengage = disengage;
    if (disengage) {
      controls_allowed = false;
    }
  }

  // Unity stalk gating on 0x45 (bus0)
  if ((bus == 0) && (addr == 0x45) && tesla_legacy_op_stalk_enable) {
    if ((!tesla_legacy_has_ap_hw) || tesla_legacy_op_autopilot_disabled) {
      const int ap_lever_position = (int)(msg->data[0] & 0x3F);
      if (ap_lever_position == 2) {
        pcm_cruise_check(true);
      } else if (ap_lever_position == 1) {
        pcm_cruise_check(false);
      } else {
      }
    }
  }

  // Stock system detection + Unity "disable when stock features active" (bus2)
  if (bus == 2) {
    if (!tesla_legacy_external_panda && (addr == 0x488)) {
      const int steer_control_type = (int)((msg->data[2] >> 6) & 0x03);
      tesla_legacy_stock_lkas = (steer_control_type == 2) || (steer_control_type == 3);
      if (tesla_legacy_stock_lkas) {
        controls_allowed = false;
      }
    } else if (tesla_legacy_external_panda && (addr == 0x2BF)) {
      const int aeb_event = (int)(msg->data[2] & 0x03);
      tesla_legacy_stock_aeb = (aeb_event == 1);
      if (tesla_legacy_stock_aeb) {
        controls_allowed = false;
      }
    } else {
    }

    if (!tesla_legacy_external_panda && tesla_legacy_has_ap_hw) {
      if (addr == 0x399) {
        const uint8_t st = msg->data[0] & 0x0FU;  // AutopilotStatus is the LOW nibble on this DBC
        tesla_legacy_autopilot_enabled = (st == 3U) || (st == 4U) || (st == 5U);
        if (tesla_legacy_autopilot_enabled) {
          controls_allowed = false;
        }
      } else if (addr == 0x219) {
        const int eac_status = (int)((msg->data[1] >> 2) & 0x0FU);
        const int psc_status = (int)((msg->data[0] >> 1) & 0x1FU);
        tesla_legacy_eac_enabled = (eac_status == 2) || (eac_status == 3);
        tesla_legacy_autopark_enabled = (psc_status == 14) || ((psc_status >= 1) && (psc_status <= 8));
        if (tesla_legacy_autopilot_enabled || tesla_legacy_eac_enabled || tesla_legacy_autopark_enabled) {
          controls_allowed = false;
        }
      } else {
      }
    }
  }

  tesla_legacy_track_controls_allowed_edge();
}

// --- TX hook ---
static bool tesla_legacy_tx_hook(const CANPacket_t *msg) {
  const int addr = (int)msg->addr;

  // Internal carrier (0x659): consume + block from CAN
  if (addr == 0x659) {
    const uint8_t b5 = msg->data[5];
    tesla_legacy_pedal_enabled = (b5 & 0x20U) != 0U;
    tesla_legacy_op_autopilot_disabled = (b5 & 0x80U) != 0U;
    tesla_legacy_op_stalk_main_edge = (b5 & 0x02U) != 0U;
    tesla_legacy_op_stalk_cancel_edge = (b5 & 0x01U) != 0U;
        if (tesla_legacy_op_stalk_enable && (!tesla_legacy_has_ap_hw || tesla_legacy_op_autopilot_disabled)) {
      if (tesla_legacy_op_stalk_main_edge) {
        pcm_cruise_check(true);
      }
      if (tesla_legacy_op_stalk_cancel_edge) {
        pcm_cruise_check(false);
      }
    }
    return false;
  }

  // Unity parity: on AP hardware cars, block OP actuation unless stock AP is disabled
  if (tesla_legacy_has_ap_hw && !tesla_legacy_op_autopilot_disabled) {
    if ((addr == 0x488) || (addr == 0x27D) || (addr == 0x2BF)) {
      return false;
    }
  }

  // Main panda (lateral)
  if (!tesla_legacy_external_panda) {
    if (addr == 0x2BF) {
      return false;
    }

    if ((addr == 0x488) || (addr == 0x27D)) {
      if (tesla_legacy_stock_lkas) {
        return false;
      }
    }

    if (addr == 0x488) {
      const int steer_control_type = (int)((msg->data[2] >> 6) & 0x03);
      const bool steer_control_enabled = (steer_control_type == 1);

      if ((steer_control_type != 0) && (steer_control_type != 1)) {
        return false;
      }
      if (steer_control_type == 0) {
        return true;
      }

      const int raw_angle_can = (((int)(msg->data[0] & 0x7FU)) << 8) | (int)msg->data[1];
      const int desired_angle = raw_angle_can - 16384;

      if (!controls_allowed) {
        if (!tesla_legacy_hands_on) {
          return false;
        }
        if ((desired_angle < (angle_meas.min - 1)) || (desired_angle > (angle_meas.max + 1))) {
          return false;
        }
        return true;
      }

      const AngleSteeringLimits limits = {
        .max_angle = 3600,
        .angle_deg_to_can = 10.0f,
        .frequency = 50U,
      };

      const AngleSteeringParams params = {
        .slip_factor = -0.000750000000000000f,
        .steer_ratio = 16.5f,
        .wheelbase = 2.96f,
      };

      return !steer_angle_cmd_checks_vm(desired_angle, steer_control_enabled, limits, params);
    }

    return true;
  }

  // External panda (longitudinal)
  if (tesla_legacy_external_panda) {
    if (addr != 0x2BF) {
      return false;
    }

    const int aeb_event = (int)(msg->data[2] & 0x03);
    if (aeb_event != 0) {
      return false;
    }
    if (tesla_legacy_stock_aeb) {
      return false;
    }

    const LongitudinalLimits limits = {.max_accel = 425, .min_accel = 288, .inactive_accel = 375};

    const int raw_accel_max = (((int)(msg->data[6] & 0x1FU)) << 4) | ((int)(msg->data[5] >> 4));
    const int raw_accel_min = (((int)(msg->data[5] & 0x0FU)) << 5) | ((int)(msg->data[4] >> 3));

    const bool long_active = (raw_accel_max != limits.inactive_accel) || (raw_accel_min != limits.inactive_accel);
    if (long_active && (!controls_allowed || !get_longitudinal_allowed())) {
      return false;
    }

    if ((raw_accel_max < limits.inactive_accel) && (raw_accel_min < limits.inactive_accel)) {
      return false;
    }

    if (longitudinal_accel_checks(raw_accel_max, limits)) {
      return false;
    }
    if (longitudinal_accel_checks(raw_accel_min, limits)) {
      return false;
    }

    return true;
  }

  return true;
}

// fwd_hook/fwd_msg return true => block forwarding
static bool tesla_legacy_fwd_hook(int bus_num, int addr) {
  (void)bus_num;
  return addr == 0x659;
}

static bool tesla_legacy_fwd_msg_hook(int bus_num, CANPacket_t *to_fwd) {
  const int addr = (int)to_fwd->addr;

  // Never forward internal carrier
  if (addr == 0x659) {
    return true;
  }

  // Prevent OP actuation echoes back to AP side
  if ((bus_num == 0) && ((addr == 0x488) || (addr == 0x27D) || (addr == 0x2BF))) {
    return true;
  }

  // External panda: only passthrough stock AEB long control (bus2 -> car)
  if (tesla_legacy_external_panda) {
    if ((bus_num == 2) && (addr == 0x2BF)) {
      return !tesla_legacy_stock_aeb;
    }
    return true;
  }

  // Main panda: stock LKAS passthrough (bus2 -> car)
  if ((bus_num == 2) && ((addr == 0x488) || (addr == 0x27D))) {
    return !tesla_legacy_stock_lkas;
  }

  // Unity mods only on main panda with AP HW
  if (!tesla_legacy_has_ap_hw) {
        if (tesla_legacy_op_stalk_enable && (!tesla_legacy_has_ap_hw || tesla_legacy_op_autopilot_disabled)) {
      if (tesla_legacy_op_stalk_main_edge) {
        pcm_cruise_check(true);
      }
      if (tesla_legacy_op_stalk_cancel_edge) {
        pcm_cruise_check(false);
      }
    }
    return false;
  }

  // bus0 -> bus2: block IC/HUD frames + eacStatus workaround
  if (bus_num == 0) {
    if ((addr == 0x399) || (addr == 0x389) || (addr == 0x239) || (addr == 0x309) ||
        (addr == 0x3A9) || (addr == 0x329) || (addr == 0x349) || (addr == 0x369)) {
      return true;
    }

    if (addr == 0x370) {
      const uint8_t b6 = to_fwd->data[6];
      const uint8_t eac_status = (b6 >> 5) & 0x07U;
      if (controls_allowed && !(tesla_legacy_autopilot_enabled || tesla_legacy_eac_enabled || tesla_legacy_autopark_enabled) && (eac_status == 2U)) {
        to_fwd->data[6] = (uint8_t)((b6 & 0x1FU) | (1U << 5));
        tesla_legacy_set_last_byte_checksum(to_fwd);
      }
    }

    return false;
  }

  // bus2 -> bus0: mutate forwarded HUD status for OP ownership on legacy AP cars
  if (bus_num == 2) {
    // Active OP ownership: keep stock speed-limit fields, only lift status/failure bits.
    if (controls_allowed && tesla_legacy_op_autopilot_disabled &&
        !tesla_legacy_autopilot_enabled && !tesla_legacy_eac_enabled && !tesla_legacy_autopark_enabled) {
      if (addr == 0x389) {
        uint32_t w0 = (uint32_t)GET_BYTES(to_fwd, 0, 4);
        uint32_t w1 = (uint32_t)GET_BYTES(to_fwd, 4, 4);
        w0 &= 0xFFFF3FFFU;   // DAS_activationFailureStatus = 0
        w1 &= 0x00FFFFFFU;   // clear checksum byte before recompute
        to_fwd->data[0] = (uint8_t)(w0 & 0xFFU);
        to_fwd->data[1] = (uint8_t)((w0 >> 8) & 0xFFU);
        to_fwd->data[2] = (uint8_t)((w0 >> 16) & 0xFFU);
        to_fwd->data[3] = (uint8_t)((w0 >> 24) & 0xFFU);
        to_fwd->data[4] = (uint8_t)(w1 & 0xFFU);
        to_fwd->data[5] = (uint8_t)((w1 >> 8) & 0xFFU);
        to_fwd->data[6] = (uint8_t)((w1 >> 16) & 0xFFU);
        to_fwd->data[7] = (uint8_t)((w1 >> 24) & 0xFFU);
        tesla_legacy_set_last_byte_checksum(to_fwd);
      } else if (addr == 0x399) {
        // AutopilotStatus is the LOW nibble on this DBC. Unity HUD used state 5 when enabled.
        to_fwd->data[0] = (to_fwd->data[0] & 0xF0U) | 0x05U;
        tesla_legacy_set_last_byte_checksum(to_fwd);
      } else {
      }
    }

    // Just after disengage: keep the existing warning-hide behavior.
    if (tesla_legacy_hide_errors_armed && !controls_allowed && !tesla_legacy_autopilot_enabled) {
      const uint32_t dt = get_ts_elapsed(microsecond_timer_get(), tesla_legacy_time_op_disengaged);
      if (dt <= TESLA_LEGACY_TIME_TO_HIDE_ERRORS_US) {
        if (addr == 0x389) {
          uint32_t w0 = (uint32_t)GET_BYTES(to_fwd, 0, 4);
          uint32_t w1 = (uint32_t)GET_BYTES(to_fwd, 4, 4);
          w0 &= 0xFFFF3FFFU;
          w1 &= 0x00FFFFFFU;
          to_fwd->data[0] = (uint8_t)(w0 & 0xFFU);
          to_fwd->data[1] = (uint8_t)((w0 >> 8) & 0xFFU);
          to_fwd->data[2] = (uint8_t)((w0 >> 16) & 0xFFU);
          to_fwd->data[3] = (uint8_t)((w0 >> 24) & 0xFFU);
          to_fwd->data[4] = (uint8_t)(w1 & 0xFFU);
          to_fwd->data[5] = (uint8_t)((w1 >> 8) & 0xFFU);
          to_fwd->data[6] = (uint8_t)((w1 >> 16) & 0xFFU);
          to_fwd->data[7] = (uint8_t)((w1 >> 24) & 0xFFU);
          tesla_legacy_set_last_byte_checksum(to_fwd);
        } else if (addr == 0x399) {
          to_fwd->data[0] = (to_fwd->data[0] & 0xF0U) | 0x02U;
          tesla_legacy_set_last_byte_checksum(to_fwd);
        } else if ((addr == 0x329) || (addr == 0x349) || (addr == 0x369)) {
          for (int i = 0; i < GET_LEN(to_fwd); i++) {
            to_fwd->data[i] = 0U;
          }
          tesla_legacy_set_last_byte_checksum(to_fwd);
        } else {
        }
      } else {
        tesla_legacy_hide_errors_armed = false;
      }
    }
    return false;
  }

  return false;
}

// --- init ---
static safety_config tesla_legacy_init(uint16_t param) {
  tesla_legacy_external_panda = GET_FLAG(param, TESLA_LEGACY_FLAG_EXTERNAL_PANDA);
  tesla_legacy_op_stalk_enable = GET_FLAG(param, TESLA_LEGACY_FLAG_OP_STALK_ENABLE);
  tesla_legacy_has_ap_hw = GET_FLAG(param, TESLA_LEGACY_FLAG_HW2) || GET_FLAG(param, TESLA_LEGACY_FLAG_HW3);
  tesla_legacy_chassis_bus = GET_FLAG(param, TESLA_LEGACY_FLAG_HW3) ? 1 : 0;

  tesla_legacy_op_autopilot_disabled = false;
  tesla_legacy_pedal_enabled = false;

  tesla_legacy_autopilot_enabled = false;
  tesla_legacy_eac_enabled = false;
  tesla_legacy_autopark_enabled = false;

  tesla_legacy_hands_on = false;
  tesla_legacy_hands_on_last_signal = 0U;

  tesla_legacy_time_op_disengaged = 0U;
  tesla_legacy_controls_allowed_prev = false;
  tesla_legacy_hide_errors_armed = false;
  tesla_legacy_in_reverse = false;
  tesla_legacy_last_gear = 0U;

  cruise_engaged_prev = false;

  static const CanMsg TESLA_LEGACY_TX_MSGS_LATERAL[] = {
    {0x488, 0, 4, .check_relay = false},  // DAS_steeringControl
    {0x27D, 0, 3, .check_relay = false},  // APS_eacMonitor
    {0x659, 0, 8, .check_relay = false},  // OP->safety internal carrier (blocked in tx_hook)
    {0x45, 0, 8, .check_relay = false},  // STW_ACTN_RQ

  };

  static const CanMsg TESLA_LEGACY_TX_MSGS_LONG[] = {
    {0x2BF, 0, 8, .check_relay = false},  // DAS_longControl
    {0x659, 0, 8, .check_relay = false},  // OP->safety internal carrier (blocked in tx_hook)
    {0x45, 0, 8, .check_relay = false},  // STW_ACTN_RQ

  };

  // These RX checks are the primary source of 'safetyRxChecksInvalid' -> controls mismatch.
  // Keep them to frames that are always present on the respective panda's wiring.
  //
  // Your canmap (split) shows:
  //  - main panda: chassis frames on bus0, AP/DAS frames on bus2
  //  - external panda: does not consistently see the AP/DAS bus; keep minimal
  static RxCheck tesla_legacy_rx_checks_external[] = {
  // Unity parity (powertrain / longitudinal): these are always present on the PT bus.
  // Keep this minimal; missing frames == rx_checks_invalid == controls mismatch.
  {.msg = {
    {0x106, 0, 8, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true},  // DI_torque1
    {0x106, 2, 8, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true},  // mirrored
    {0},
  }},
  {.msg = {
    {0x116, 0, 6, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true},  // DI_torque2
    {0x116, 2, 6, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true},  // mirrored
    {0},
  }},
};

  static RxCheck tesla_legacy_rx_checks_main[] = {
  // Unity parity (AP-side / lateral): torque frames are the most reliable "always present" signals.
  {.msg = {
    {0x108, 0, 8, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true},  // DI_torque1
    {0x108, 2, 8, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true},  // mirrored
    {0},
  }},
  {.msg = {
    {0x118, 0, 6, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true},  // DI_torque2
    {0x118, 2, 6, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true},  // mirrored
    {0},
  }},
};

  safety_config ret = tesla_legacy_external_panda
    ? BUILD_SAFETY_CFG(tesla_legacy_rx_checks_external, TESLA_LEGACY_TX_MSGS_LONG)
    : BUILD_SAFETY_CFG(tesla_legacy_rx_checks_main, TESLA_LEGACY_TX_MSGS_LATERAL);

  // Critical for XNOR: external panda must NOT forward.
  // Main panda forwards only when AP HW exists.
  ret.disable_forwarding = tesla_legacy_external_panda || !tesla_legacy_has_ap_hw;
  return ret;
}

const safety_hooks tesla_legacy_hooks = {
  .init = tesla_legacy_init,
  .rx = tesla_legacy_rx_hook,
  .tx = tesla_legacy_tx_hook,
  .fwd = tesla_legacy_fwd_hook,
  .fwd_msg = tesla_legacy_fwd_msg_hook,
};
