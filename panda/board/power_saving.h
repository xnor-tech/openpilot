#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "board/sys/sys.h"
#include "board/power_saving_declarations.h"

// F4-compatible power-saving implementation aligned to xnor-dev's bool API.
bool power_save_enabled = false;
#ifdef ALLOW_DEBUG
volatile bool stop_mode_requested = false;
#endif

void enable_can_transceivers(bool enabled) {
  // Leave main CAN always on for CAN-based ignition detection
  uint8_t main_bus = (harness.status == HARNESS_STATUS_FLIPPED) ? 3U : 1U;
  for (uint8_t i = 1U; i <= 4U; i++) {
    current_board->enable_can_transceiver(i, (i == main_bus) || enabled);
  }
}

void set_power_save_state(bool enable) {
  if (enable != power_save_enabled) {
    if (enable) {
      print("enable power savings\n");

      // Disable CAN interrupts.
      if (harness.status == HARNESS_STATUS_FLIPPED) {
        llcan_irq_disable(cans[0]);
      } else {
        llcan_irq_disable(cans[2]);
      }
      llcan_irq_disable(cans[1]);
    } else {
      print("disable power savings\n");

      if (harness.status == HARNESS_STATUS_FLIPPED) {
        llcan_irq_enable(cans[0]);
      } else {
        llcan_irq_enable(cans[2]);
      }
      llcan_irq_enable(cans[1]);
    }

    enable_can_transceivers(!enable);

    // Switch off IR when in power saving.
    if (enable) {
      current_board->set_ir_power(0U);
    }

    power_save_enabled = enable;
  }
}
