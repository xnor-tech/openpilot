#pragma once

#include "board/drivers/bxcan_declarations.h"

bool llbxcan_set_speed(CAN_TypeDef *CANx, uint32_t speed, bool loopback, bool silent) {
  UNUSED(loopback);
  UNUSED(silent);
  return llcan_set_speed(CANx, speed, false, false);
}

void can_init_all(void) {
  bool silent = can_silent;
  if (!silent) {
    silent = all_can_silent;
  }
  for (uint8_t i = 0U; i < CAN_MAX; i++) {
    if (can_enabled(i)) {
      bool ret = llbxcan_init(can_hw[i], i) && llbxcan_set_speed(can_hw[i], can_speed[i], false, silent);
      UNUSED(ret);
    }
  }
}

void can_set_mode(uint8_t mode) {
  bool silent = (mode == CAN_MODE_SILENT);
  can_silent = silent;
  can_init_all();
}

void process_can(uint8_t can_number) {
  if (can_number < CAN_MAX) {
    CAN_TypeDef *CANx = can_hw[can_number];

    while ((CANx->RF0R & CAN_RF0R_FMP0) != 0U) {
      pending_can_live = 1U;

      CANPacket_t to_push;
      to_push.fd = 0U;
      to_push.returned = 0U;
      to_push.rejected = 0U;
      to_push.extended = (CANx->sFIFOMailBox[0].RIR >> 2) & 0x1U;
      to_push.addr = (to_push.extended != 0U) ? (CANx->sFIFOMailBox[0].RIR >> 3) : (CANx->sFIFOMailBox[0].RIR >> 21);
      to_push.data_len_code = CANx->sFIFOMailBox[0].RDTR & 0xFU;
      to_push.bus = can_number;
      WORD_TO_BYTE_ARRAY(&to_push.data[0], CANx->sFIFOMailBox[0].RDLR);
      WORD_TO_BYTE_ARRAY(&to_push.data[4], CANx->sFIFOMailBox[0].RDHR);
      can_set_checksum(&to_push);

      CANPacket_t to_send = to_push;
      to_send.returned = 0U;
      to_send.rejected = 0U;
      int bus_fwd_num = safety_fwd_hook(can_number, &to_send);
      if (bus_fwd_num != -1) {
        to_send.bus = (uint8_t)bus_fwd_num;
        can_set_checksum(&to_send);
        can_send(&to_send, (uint8_t)bus_fwd_num, true);
        can_health[can_number].total_fwd_cnt += 1U;
      }

      safety_rx_invalid += safety_rx_hook(&to_push) ? 0U : 1U;
      ignition_can_hook(&to_push);

      led_set(LED_BLUE, true);
      rx_buffer_overflow += can_push(&can_rx_q, &to_push) ? 0U : 1U;

      CANx->RF0R |= CAN_RF0R_RFOM0;
    }
  }
}
