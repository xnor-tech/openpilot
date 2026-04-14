from opendbc.can import CANParser
from opendbc.car import Bus, structs
from opendbc.car.interfaces import RadarInterfaceBase
from opendbc.car.tesla.values import DBC, CAR, CANBUS

RADAR_START_ADDR = 0x410
RADAR_MSG_COUNT = 80  # 40 points * 2 messages each

# Bosch radar is only on specific older vehicles and uses different CAN messages
BOSCH_RADAR_CARS = (CAR.TESLA_MODEL_S_HW1, CAR.TESLA_MODEL_S_HW2, CAR.TESLA_MODEL_X_HW1, CAR.TESLA_MODEL_X_HW2)
BOSCH_NUM_POINTS = 32
BOSCH_RADAR_POINT_FRQ = 8
BOSCH_TRIGGER_MSG = 878


def _is_bosch_radar(CP):
  return CP.carFingerprint in BOSCH_RADAR_CARS


def get_radar_can_parser(CP):
  if Bus.radar not in DBC[CP.carFingerprint]:
    return None

  is_bosch = _is_bosch_radar(CP)

  if is_bosch:
    messages = [('TeslaRadarSguInfo', 8)]
    num_points = BOSCH_NUM_POINTS
    freq = BOSCH_RADAR_POINT_FRQ
  else:
    messages = [('RadarStatus', 16)]
    num_points = RADAR_MSG_COUNT // 2
    freq = 16

  for i in range(num_points):
    messages.extend([
      (f'RadarPoint{i}_A', freq),
      (f'RadarPoint{i}_B', freq),
    ])

  return CANParser(DBC[CP.carFingerprint][Bus.radar], messages, CANBUS.radar)


class RadarInterface(RadarInterfaceBase):
  def __init__(self, CP):
    super().__init__(CP)
    self.updated_messages = set()
    self.track_id = 0

    self.bosch_radar = _is_bosch_radar(CP)
    self.continental_radar = not self.bosch_radar and not CP.radarUnavailable

    if self.bosch_radar:
      self.trigger_msg = BOSCH_TRIGGER_MSG
      self.num_points = BOSCH_NUM_POINTS
    else:
      self.trigger_msg = RADAR_START_ADDR + RADAR_MSG_COUNT - 1
      self.num_points = RADAR_MSG_COUNT // 2

    self.radar_off_can = CP.radarUnavailable
    self.rcp = get_radar_can_parser(CP)

  def update(self, can_strings):
    if self.radar_off_can or self.rcp is None:
      return super().update(None)

    vls = self.rcp.update(can_strings)
    self.updated_messages.update(vls)

    ret = structs.RadarData()
    if self.trigger_msg not in self.updated_messages:
      return ret

    rr = self._update(self.updated_messages)
    self.updated_messages.clear()

    return rr

  def _update(self, updated_messages):
    ret = structs.RadarData()
    if self.rcp is None:
      return ret

    if not self.rcp.can_valid:
      ret.errors.canError = True

    if self.continental_radar:
      radar_status = self.rcp.vl['RadarStatus']
      if radar_status['shortTermUnavailable']:
        ret.errors.radarUnavailableTemporary = True
      if radar_status['sensorBlocked'] or radar_status['vehDynamicsError']:
        ret.errors.radarFault = True
    elif self.bosch_radar:
      radar_status = self.rcp.vl['TeslaRadarSguInfo']
      if radar_status['RADC_HWFail']:
        ret.errors.radarFault = True

    for i in range(self.num_points):
      msg_a = self.rcp.vl[f'RadarPoint{i}_A']
      msg_b = self.rcp.vl[f'RadarPoint{i}_B']

      # Make sure msg A and B are together
      if msg_a['Index'] != msg_b['Index2']:
        continue

      if not msg_a['Tracked']:
        if i in self.pts:
          del self.pts[i]
        continue

      if self.bosch_radar and (msg_a['LongDist'] > 250.0 or msg_a['LongDist'] <= 0 or msg_a['ProbExist'] < 50.0):
        if i in self.pts:
          del self.pts[i]
        continue

      if i not in self.pts:
        self.pts[i] = structs.RadarData.RadarPoint()
        self.pts[i].trackId = self.track_id
        self.track_id += 1

      self.pts[i].dRel = msg_a['LongDist']
      self.pts[i].yRel = msg_a['LatDist']
      self.pts[i].vRel = msg_a['LongSpeed']
      self.pts[i].aRel = msg_a['LongAccel']
      self.pts[i].yvRel = msg_b['LatSpeed']
      self.pts[i].measured = bool(msg_a['Meas'])

    ret.points = list(self.pts.values())
    return ret
