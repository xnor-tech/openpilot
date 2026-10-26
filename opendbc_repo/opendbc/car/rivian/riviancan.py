from opendbc.car.crc import CRC8J1850


def create_lka_steering(packer, frame, acm_lka_hba_cmd, apply_torque, enabled, active, mads):
  # forward auto high beam and speed limit status and nothing else
  values = {s: acm_lka_hba_cmd[s] for s in (
    "ACM_hbaSysState",
    "ACM_hbaLamp",
    "ACM_hbaOnOffState",
    "ACM_slifOnOffState",
  )}

  values |= {
    "ACM_lkaHbaCmd_Counter": frame % 15,  # 0-14 counter (15 states), so % 15 is correct despite the 4-bit field
    "ACM_lkaStrToqReq": apply_torque,
    "ACM_lkaActToi": active,

    "ACM_lkaLaneRecogState": 3 if mads.lka_icon_states else 0,
    "ACM_lkaSymbolState": 3 if mads.lka_icon_states else 0,

    # static values
    "ACM_lkaElkRequest": 0,
    "ACM_ldwlkaOnOffState": 2,  # 2=LKAS+LDW on
    "ACM_elkOnOffState": 1,  # 1=LKAS on
    # TODO: what are these used for?
    "ACM_ldwWarnTypeState": 2,  # always 2
    "ACM_ldwWarnTimingState": 1,  # always 1
    #"ACM_lkaHandsoffDisplayWarning": 1,  # TODO: we can send this when openpilot wants you to pay attention
  }

  return packer.make_can_msg("ACM_lkaHbaCmd", 0, values)


def create_angle_steering(packer, frame, angle, active):
  values = {
    "ACM_SteeringControl_Counter": frame % 15,
    "ACM_SteeringAngleRequest": angle,
    "ACM_EacEnabled": active,
    "ACM_HapticRequired": 0
  }

  return packer.make_can_msg("ACM_SteeringControl", 0, values)

def create_acm_status(packer, frame, feature_status):
  values = {
    "ACM_Status_Counter": frame % 15,
    "ACM_FeatureStatus": feature_status,
    "ACM_FaultStatus": 0,
    "ACM_FaultSupervisorState": 0,
    "ACM_Unkown1": 0,
  }

  return packer.make_can_msg("ACM_Status", 0, values)


def create_wheel_touch(packer, sccm_wheel_touch, enabled):
  values = {s: sccm_wheel_touch[s] for s in (
    "SCCM_WheelTouch_Counter",
    "SCCM_WheelTouch_HandsOn",
    "SCCM_WheelTouch_Calibration",
    "SCCM_WheelTouch_CapacitiveValue",
    "SCCM_WheelTouch_ResistiveValue",
  )}

  # When only using ACC without lateral, the ACM warns the driver to hold the steering wheel on engagement
  # Tell the ACM that the user is holding the wheel to avoid this warning
  if enabled:
    values["SCCM_WheelTouch_HandsOn"] = 1
    values["SCCM_WheelTouch_CapacitiveValue"] = 100  # only need to send this value, but both are set for consistency

  return packer.make_can_msg("SCCM_WheelTouch", 2, values)


def create_longitudinal(packer, counter, accel, enabled):
  values = {
    "ACM_longitudinalRequest_Counter": counter % 15,  # 0-14 counter (15 states), so % 15 is correct despite the 4-bit field
    "ACM_AccelerationRequest": accel,
    "ACM_PrndRequest": 0,
    "ACM_longInterfaceEnable": 1 if enabled else 0,
    "ACM_VehicleHoldRequest": 0,
  }

  return packer.make_can_msg("ACM_longitudinalRequest", 0, values)


FCM_FAKE_LEAD_ID = 43


def create_camera_lead_status_a(packer, frame, camera_lead_status_a):
  # camera lead frame with defaults applied
  values = dict(camera_lead_status_a)
  values["COUNTER"] = frame % 4
  values["LEAD_LOST_STATE"] = 0
  values["ID_LEAD"] = values["ID_LEAD"] or FCM_FAKE_LEAD_ID
  values["VIDEO_OBJECT_COUNT"] = values["VIDEO_OBJECT_COUNT"] or 1
  values["VIDEO_VEHICLE_COUNT"] = values["VIDEO_VEHICLE_COUNT"] or 1
  return packer.make_can_msg("CAMERA_LEAD_STATUS_A", 1, values)


def create_camera_lead_status_b(packer, frame, camera_lead_status_b):
  values = dict(camera_lead_status_b)
  values["LEAD_PRESENT"] = 1
  values["COUNTER"] = frame % 4
  return packer.make_can_msg("CAMERA_LEAD_STATUS_B", 1, values)


def create_adas_status(packer, vdm_adas_status, interface_status, driver_override=None, block_engage=False):
  values = {s: vdm_adas_status[s] for s in (
    "VDM_AdasStatus_Counter",
    "VDM_AdasDecelLimit",
    "VDM_AdasDriverAccelPriorityStatus",
    "VDM_AdasFaultStatus",
    "VDM_AdasAccelLimit",
    "VDM_AdasDriverModeStatus",
    "VDM_AdasUnkown1",
    "VDM_AdasInterfaceStatus",
    "VDM_AdasVehicleHoldStatus",
    "VDM_UserAdasRequest",
  )}

  # VDM_UserAdasRequest: 0=IDLE, 1=UP_1 (cancel), 2=UP_2, 3=DOWN_1, 4=DOWN_2. Only let idle/cancel through.
  if block_engage and values["VDM_UserAdasRequest"] not in (0, 1):
    values["VDM_UserAdasRequest"] = 0

  if interface_status is not None:
    if interface_status == 1:
      values["VDM_UserAdasRequest"] = 1
    values["VDM_AdasInterfaceStatus"] = interface_status

  if driver_override is not None:
    values["VDM_AdasDriverModeStatus"] = driver_override

  return packer.make_can_msg("VDM_AdasSts", 2, values)


def rivian_checksum(address: int, sig, d: bytearray) -> int:
  # CRC-8 with the little-endian CAN address prepended to the payload.
  crc = 0
  for byte in address.to_bytes(2, "little") + d[1:]:
    crc = CRC8J1850[crc ^ byte]
  return crc
