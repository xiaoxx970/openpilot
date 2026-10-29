from openpilot.common.params import Params, UnknownKeyName
from openpilot.system.ui.widgets import Widget
from openpilot.system.ui.widgets.list_view import multiple_button_item, toggle_item
from openpilot.system.ui.widgets.scroller_tici import Scroller
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.lib.multilang import tr, tr_noop
from openpilot.selfdrive.ui.ui_state import ui_state

if gui_app.sunnypilot_ui():
  from openpilot.system.ui.sunnypilot.widgets.list_view import toggle_item_sp as toggle_item
  from openpilot.system.ui.sunnypilot.widgets.list_view import multiple_button_item_sp as multiple_button_item

ROAD_EVENT_PROMPTS = 3  # VwRoadEventPrompts: off, when engaged, always

# Description constants
DESCRIPTIONS = {
  "EnableCurvatureController": tr_noop(
    "Enables curvature PID post-processing additionally to QFK curvature offset"
  ),
  "EnableCurvatureD": tr_noop(
    "Learns speed- and curvature-dependent steering corrections around center for dynamic steering behavior. " +
    "Experimental and only used on curvature-based steering paths."
  ),
  "ShowDynamicSteeringLearnerGraph": tr_noop(
    "Display the current dynamic steering learner fit, marker, and status information in the onroad UI."
  ),
  "EnableLongComfortMode": tr_noop(
    "Enables longitudinal jerk and accel deviation limit control for safe and comfortable driving"
  ),
  "EnableSpeedLimitControl": tr_noop(
    "Enables setting maximum speed by speed limit detection"
  ),
  "VwVisionCarTypes": tr_noop(
    "Shows trucks, two-wheelers and people as such on the cluster's lead and side lane cars. " +
    "Needs the model file in /data/yolo_lead; uses an idle CPU core, never the GPU."
  ),
  "RainFollowDistance": tr_noop(
    "When the rain sensor or wiper stalk says it rains, switch to a rain following distance one step past relaxed " +
    "(1.75 s, more above 80 km/h) and back when it stops. The distance button also reaches it while it rains."
  ),
  "CurveSpeedDriverProfile": tr_noop(
    "Curve speed from your own driving instead of Smart Cruise Control (Vision)'s fixed limit: lateral accel by speed, " +
    "braking planned ahead of the curve from the camera and the offline map's road, a steady target inside it and " +
    "speeding up out of it at your pace. Needs Smart Cruise Control (Vision) on; off gives exactly the stock curve control."
  ),
  "VwRoadEventPrompts": tr_noop(
    "Shows uphill / downhill ahead from the driving model, roundabout / crossing ahead from the offline map " +
    "(download it in the OSM settings) and with Baidu Traffic a traffic jam ahead on the cluster: when engaged while " +
    "openpilot's ACC is on, always also in ACC standby and with the main switch off. Display only, except that with Baidu " +
    "Traffic a traffic jam ahead selects the relaxed following distance until the car reaches it, like rain's."
  ),
  "VwBaiduOnline": tr_noop(
    "Baidu Maps' live data for the road ahead: speed limits in place of the offline map's (Speed Limit Assist still asks " +
    "before each change), and with Road Event Prompts a traffic jam ahead, shown and approached at the relaxed following " +
    "distance. Sends the car's recent positions to Baidu a few times a minute over mobile data. Needs your Baidu Maps key " +
    "in /data/baidu_ak."
  ),
  "EnableSpeedLimitPredicative": tr_noop(
    "Enables setting predicative speed limit"
  ),
  "EnableSLPredReactToSL": tr_noop(
    "Enables reaction to speed limits as predicative speed limit"
  ),
  "EnableSLPredReactToCurves": tr_noop(
    "Enables reaction to curves as predicative speed limit (Max Speed as per lateral ISO limits)"
  ),
  "BatteryDetails": tr_noop(
    "Display battery detail panel"
  ),
  "ForceRHDForBSM": tr_noop(
    "Switch BSM detection side to RHD. Driver is on the right side."
  ),
  "EnableSmoothSteer": tr_noop(
    "Enables S-curving on lateral control for smoother steering"
  ),
  "DisableCarSteerAlerts": tr_noop(
    "Disables audible steering alerts from car"
  ),
}


class ICTogglesLayout(Widget):
  def __init__(self):
    super().__init__()
    self._params = Params()

    # param, title, desc, icon, needs_restart
    self._toggle_defs = {
      "EnableCurvatureController": (
        lambda: tr("VW: Lateral Correction (Recommended)"),
        DESCRIPTIONS["EnableCurvatureController"],
        "chffr_wheel.png",
        False,
      ),
      "EnableLongComfortMode": (
        lambda: tr("VW: Longitudinal Comfort Mode"),
        DESCRIPTIONS["EnableLongComfortMode"],
        "chffr_wheel.png",
        False,
      ),
      "EnableSpeedLimitControl": (
        lambda: tr("VW: Speed Limit Control"),
        DESCRIPTIONS["EnableSpeedLimitControl"],
        "speed_limit.png",
        False,
      ),
      "VwVisionCarTypes": (
        lambda: tr("VW: Car Types on Cluster"),
        DESCRIPTIONS["VwVisionCarTypes"],
        "chffr_wheel.png",
        False,
      ),
      "RainFollowDistance": (
        lambda: tr("VW: Rain Following Distance"),
        DESCRIPTIONS["RainFollowDistance"],
        "chffr_wheel.png",
        False,
      ),
      "CurveSpeedDriverProfile": (
        lambda: tr("VW: Curve Speed from Driver Profile"),
        DESCRIPTIONS["CurveSpeedDriverProfile"],
        "chffr_wheel.png",
        False,
      ),
      "VwBaiduOnline": (
        lambda: tr("VW: Baidu Traffic & Speed Limits"),
        DESCRIPTIONS["VwBaiduOnline"],
        "chffr_wheel.png",
        False,
      ),
      "EnableSpeedLimitPredicative": (
        lambda: tr("VW: Predicative Speed Limit (pACC)"),
        DESCRIPTIONS["EnableSpeedLimitPredicative"],
        "speed_limit.png",
        False,
      ),
      "EnableSLPredReactToSL": (
        lambda: tr("VW: Predicative - Reaction to Speed Limits"),
        DESCRIPTIONS["EnableSLPredReactToSL"],
        "speed_limit.png",
        False,
      ),
      "EnableSLPredReactToCurves": (
        lambda: tr("VW: Predicative - Reaction to Curves"),
        DESCRIPTIONS["EnableSLPredReactToCurves"],
        "speed_limit.png",
        False,
      ),
      "ForceRHDForBSM": (
        lambda: tr("VW: Force RHD for BSM"),
        DESCRIPTIONS["ForceRHDForBSM"],
        "chffr_wheel.png",
        False,
      ),
      "DisableCarSteerAlerts": (
        lambda: tr("VW: Disable Car Steer Alert Chime"),
        DESCRIPTIONS["DisableCarSteerAlerts"],
        "chffr_wheel.png",
        False,
      ),
      "EnableSmoothSteer": (
        lambda: tr("Steer Smoothing"),
        DESCRIPTIONS["EnableSmoothSteer"],
        "chffr_wheel.png",
        False,
      ),
      "BatteryDetails": (
        lambda: tr("VW MEB: Display Battery Details"),
        DESCRIPTIONS["BatteryDetails"],
        "capslock-fill.png",
        False,
      ),
      "EnableCurvatureD": (
        lambda: tr("Enable Dynamic Steering Learner"),
        DESCRIPTIONS["EnableCurvatureD"],
        "chffr_wheel.png",
        False,
      ),
      "ShowDynamicSteeringLearnerGraph": (
        lambda: tr("Show Dynamic Steering Learner Graph"),
        DESCRIPTIONS["ShowDynamicSteeringLearnerGraph"],
        "chffr_wheel.png",
        False,
      ),
    }

    self._road_event_prompts = multiple_button_item(
      lambda: tr("VW: Road Event Prompts"),
      lambda: tr(DESCRIPTIONS["VwRoadEventPrompts"]),
      buttons=[lambda: tr("Off"), lambda: tr("When Engaged"), lambda: tr("Always")],
      button_width=300,
      callback=lambda index: self._params.put("VwRoadEventPrompts", index),
      selected_index=self._road_event_prompts_index(),
      icon="chffr_wheel.png",
    )

    self._toggles = {}
    self._locked_toggles = set()
    self._offroad_only_toggles = {"EnableCurvatureD"}
    for param, (title, desc, icon, needs_restart) in self._toggle_defs.items():
      # the road prompts' three states before Baidu, which they share the jam with
      if param == "VwBaiduOnline":
        self._toggles["VwRoadEventPrompts"] = self._road_event_prompts
      toggle = toggle_item(
        title,
        desc,
        self._params.get_bool(param),
        callback=lambda state, p=param: self._toggle_callback(state, p),
        icon=icon,
      )

      try:
        locked = self._params.get_bool(param + "Lock")
      except UnknownKeyName:
        locked = False
      toggle.action_item.set_enabled(not locked)

      # Make description callable for live translation
      additional_desc = ""
      if needs_restart and not locked:
        additional_desc = tr("Changing this setting will restart openpilot if the car is powered on.")
      toggle.set_description(lambda og_desc=toggle.description, add_desc=additional_desc: tr(og_desc) + (" " + tr(add_desc) if add_desc else ""))

      # track for engaged state updates
      if locked:
        self._locked_toggles.add(param)

      self._toggles[param] = toggle

    self._scroller = Scroller(list(self._toggles.values()), line_separator=True, spacing=0)

    ui_state.add_engaged_transition_callback(self._update_toggles)
    ui_state.add_offroad_transition_callback(self._update_toggles)

  def _update_state(self):
    return

  def show_event(self):
    super().show_event()
    self._scroller.show_event()
    self._update_toggles()

  def _update_toggles(self):
    # Use ui_state's params cache (refreshed in own thread at 5Hz) to avoid extra IPC roundtrips
    ui_state.update_params()

    # TODO: make a param control list item so we don't need to manage internal state as much here
    # refresh toggles from params to mirror external changes
    for param in self._toggle_defs:
      self._toggles[param].action_item.set_state(ui_state.params.get_bool(param))
    self._road_event_prompts.action_item.set_selected_button(self._road_event_prompts_index())

    # these toggles need restart, block while engaged
    for toggle_def in self._toggle_defs:
      if self._toggle_defs[toggle_def][3] and toggle_def not in self._locked_toggles:
        self._toggles[toggle_def].action_item.set_enabled(not ui_state.engaged)

    for toggle_def in self._offroad_only_toggles:
      if toggle_def not in self._locked_toggles:
        self._toggles[toggle_def].action_item.set_enabled(ui_state.is_offroad())

    if "EnableCurvatureD" not in self._locked_toggles:
      self._toggles["EnableCurvatureD"].action_item.set_enabled(ui_state.is_offroad())

  def _road_event_prompts_index(self) -> int:
    return min(max(int(self._params.get("VwRoadEventPrompts", return_default=True) or 0), 0), ROAD_EVENT_PROMPTS - 1)

  def _render(self, rect):
    self._scroller.render(rect)

  def _toggle_callback(self, state: bool, param: str):
    self._params.put_bool(param, state)
    if self._toggle_defs[param][3]:
      self._params.put_bool("OnroadCycleRequested", True)
