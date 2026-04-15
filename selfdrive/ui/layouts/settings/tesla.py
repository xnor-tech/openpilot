from __future__ import annotations

import pyray as rl

from openpilot.common.params import Params
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.widgets import DialogResult, Widget
from openpilot.system.ui.widgets.list_view import ItemAction, ListItem, button_item
from openpilot.system.ui.widgets.option_dialog import MultiOptionDialog
from openpilot.system.ui.widgets.scroller_tici import Scroller
from openpilot.system.ui.widgets.toggle import HEIGHT as TOGGLE_HEIGHT, Toggle, WIDTH as TOGGLE_WIDTH


class ParamToggleAction(ItemAction):
  def __init__(self, params: Params, key: str):
    super().__init__(width=TOGGLE_WIDTH, enabled=True)
    self._params = params
    self._key = key
    self._toggle = Toggle(initial_state=params.get_bool(key))
    self._toggle.set_touch_valid_callback(lambda: False)

  def _handle_mouse_release(self, _mouse_pos):
    new_state = not self._params.get_bool(self._key)
    self._params.put_bool(self._key, new_state)
    self._toggle.set_state(new_state)

  def _render(self, rect: rl.Rectangle) -> bool:
    state = self._params.get_bool(self._key)
    self._toggle.set_enabled(self.enabled)
    self._toggle.set_state(state)
    self._toggle.render(rl.Rectangle(rect.x, rect.y + (rect.height - TOGGLE_HEIGHT) / 2, TOGGLE_WIDTH, TOGGLE_HEIGHT))
    return False


def param_toggle_item(title: str, description: str, params: Params, key: str) -> ListItem:
  return ListItem(title=lambda: tr(title), description=lambda: tr(description), action_item=ParamToggleAction(params, key))


class TeslaLayout(Widget):
  def __init__(self):
    super().__init__()
    self._params = Params()
    self._dialog: MultiOptionDialog | None = None
    self._dialog_handler = None

    self._items = [
      button_item("Follow Distance", self._follow_distance_text, self._follow_distance_desc, callback=self._show_follow_distance),
      button_item("Hands-on Threshold", self._hands_on_text, self._hands_on_desc, callback=self._show_hands_on_level),
      button_item("Radar Offset", self._radar_offset_text, self._radar_offset_desc, callback=self._show_radar_offset),
      param_toggle_item("Match Speed to Speed Limit", "Automatically sets cruise speed to the detected speed limit with an offset.", self._params, "TinklaAdjustAccWithSpeedLimit"),
      param_toggle_item("Offset is Percentage", "If enabled, offset is a percentage of the speed limit. Otherwise it is an absolute mph/kph offset.", self._params, "TinklaSpeedLimitUseRelative"),
      button_item("Speed Limit Offset", self._speed_limit_offset_text, self._speed_limit_offset_desc, callback=self._show_speed_limit_offset),
      param_toggle_item("Auto Lane Change", "Automatically starts lane changes after a short indicator tap, if clear.", self._params, "TinklaEnableALC"),
      button_item("Auto Lane Change Delay", self._alc_delay_text, self._alc_delay_desc, callback=self._show_alc_delay),
      param_toggle_item("Radar Upside Down", "Use if your Tesla radar is mounted upside down.", self._params, "TinklaUseTeslaRadarUpsideDown"),
      param_toggle_item("Ignore Radar SGU Error", "Ignore SGU hardware fail on Tesla radar.", self._params, "TinklaTeslaRadarIgnoreSGUError"),
      param_toggle_item("Ignore Stock AEB", "Ignore Tesla stock Automatic Emergency Braking events.", self._params, "TinklaIgnoreStockAeb"),
      param_toggle_item("Autopilot Disabled", "Unity mode: steering assist when Tesla Autopilot is disabled.", self._params, "TinklaAutopilotDisabled"),
      param_toggle_item("Mute Engage/Disengage Sounds", "Disables the start and stop sounds.", self._params, "TinklaDisableStartStopSounds"),
      param_toggle_item("Mute Prompt Sounds", "Disables prompt sounds.", self._params, "TinklaDisablePromptSounds"),
    ]
    self._scroller = Scroller(self._items, line_separator=True, spacing=0)

  def _render(self, rect: rl.Rectangle):
    self._scroller.render(rect)

  def _get_float(self, key: str, default: float) -> float:
    val = self._params.get(key, return_default=True)
    try:
      if isinstance(val, (bytes, bytearray)):
        val = val.decode("utf-8", errors="ignore")
      return float(val)
    except (TypeError, ValueError):
      return float(default)

  def _follow_distance_desc(self) -> str:
    return tr("Overrides longitudinal time-gap for Tesla (seconds).")

  def _hands_on_desc(self) -> str:
    return tr("Changes steering wheel touch sensitivity (1-4).")

  def _radar_offset_desc(self) -> str:
    return tr("Adjusts Tesla radar longitudinal offset (meters).")

  def _speed_limit_offset_desc(self) -> str:
    return tr("Offset applied when matching speed limits.")

  def _alc_delay_desc(self) -> str:
    return tr("Delay before starting an automatic lane change (seconds).")

  def _follow_distance_text(self) -> str:
    return f"{max(0.5, min(3.0, self._get_float('TinklaFollowDistance', 1.45))):.2f} s"

  def _hands_on_text(self) -> str:
    return str(max(1, min(4, int(round(self._get_float('TinklaHandsOnLevel', 3.0))))))

  def _radar_offset_text(self) -> str:
    return f"{max(-1.0, min(1.0, self._get_float('TinklaRadarOffset', 0.0))):+.1f} m"

  def _speed_limit_offset_text(self) -> str:
    is_metric = self._params.get_bool("IsMetric")
    min_v = -30.0 if is_metric else -20.0
    max_v = 30.0 if is_metric else 20.0
    units = "kph" if is_metric else "mph"
    value = max(min_v, min(max_v, self._get_float("TinklaSpeedLimitOffset", 0.0)))
    return f"{value:.1f} {units}"

  def _alc_delay_text(self) -> str:
    return f"{max(0.0, min(10.0, self._get_float('TinklaAlcDelay', 2.0))):.1f} s"

  def _open_dialog(self, title: str, options: list[str], current: str, on_confirm):
    if self._dialog is not None and gui_app.widget_in_stack(self._dialog):
      return

    self._dialog_handler = on_confirm
    self._dialog = MultiOptionDialog(tr(title), options, current=current, callback=self._handle_dialog)
    gui_app.push_widget(self._dialog)

  def _handle_dialog(self, result: int):
    if result == DialogResult.CONFIRM and self._dialog is not None and self._dialog_handler is not None:
      self._dialog_handler(self._dialog.selection)
    self._dialog = None
    self._dialog_handler = None

  def _show_follow_distance(self):
    options = [f"{v:.2f} s" for v in [x / 100.0 for x in range(50, 301, 5)]]
    self._open_dialog("Follow Distance", options, self._follow_distance_text(), self._set_follow_distance)

  def _set_follow_distance(self, selection: str):
    try:
      value = float(selection.split()[0])
    except (ValueError, IndexError):
      return
    value = max(0.5, min(3.0, value))
    self._params.put("TinklaFollowDistance", f"{value:.2f}")

  def _show_hands_on_level(self):
    options = [str(v) for v in range(1, 5)]
    self._open_dialog("Hands-on Threshold", options, self._hands_on_text(), self._set_hands_on_level)

  def _set_hands_on_level(self, selection: str):
    try:
      value = int(selection)
    except ValueError:
      return
    value = max(1, min(4, value))
    self._params.put("TinklaHandsOnLevel", str(value))

  def _show_radar_offset(self):
    options = [f"{(x / 10.0):+.1f} m" for x in range(-10, 11)]
    self._open_dialog("Radar Offset", options, self._radar_offset_text(), self._set_radar_offset)

  def _set_radar_offset(self, selection: str):
    try:
      value = float(selection.split()[0])
    except (ValueError, IndexError):
      return
    value = max(-1.0, min(1.0, value))
    self._params.put("TinklaRadarOffset", f"{value:+.1f}")

  def _show_speed_limit_offset(self):
    is_metric = self._params.get_bool("IsMetric")
    min_v = -30 if is_metric else -20
    max_v = 30 if is_metric else 20
    units = "kph" if is_metric else "mph"
    options = [f"{float(v):.1f} {units}" for v in range(min_v, max_v + 1)]
    self._open_dialog("Speed Limit Offset", options, self._speed_limit_offset_text(), self._set_speed_limit_offset)

  def _set_speed_limit_offset(self, selection: str):
    try:
      value = float(selection.split()[0])
    except (ValueError, IndexError):
      return
    is_metric = self._params.get_bool("IsMetric")
    min_v = -30.0 if is_metric else -20.0
    max_v = 30.0 if is_metric else 20.0
    value = max(min_v, min(max_v, value))
    self._params.put("TinklaSpeedLimitOffset", f"{value:.1f}")

  def _show_alc_delay(self):
    options = [f"{(x / 2.0):.1f} s" for x in range(0, 21)]
    self._open_dialog("Auto Lane Change Delay", options, self._alc_delay_text(), self._set_alc_delay)

  def _set_alc_delay(self, selection: str):
    try:
      value = float(selection.split()[0])
    except (ValueError, IndexError):
      return
    value = max(0.0, min(10.0, value))
    self._params.put("TinklaAlcDelay", f"{value:.1f}")
