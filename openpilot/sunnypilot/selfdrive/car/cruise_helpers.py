"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from openpilot.cereal import custom
from opendbc.car.structs import car
from opendbc.car import structs
from openpilot.common.params import Params

ButtonType = car.CarState.ButtonEvent.Type
EventNameSP = custom.OnroadEventSP.EventName

DISTANCE_LONG_PRESS = 50

# DEC state set by the last long press. Params writes are asynchronous, so the alert built in the same
# frame reads this instead of reading the param back.
last_dec_switch = False


class CruiseHelper:
  def __init__(self, CP: structs.CarParams):
    self.CP = CP
    self.params = Params()

    self.button_frame_counts = {ButtonType.gapAdjustCruise: 0}
    self._dynamic_experimental_control = False
    # set once the long press has acted, so its release does not also step the following distance
    self.experimental_mode_switched = False

  def update(self, CS, events, experimental_mode) -> None:
    if self.CP.openpilotLongitudinalControl:
      if CS.cruiseState.available:
        self.update_button_frame_counts(CS)

        # toggle Dynamic Experimental Control once on distance button hold
        self.update_dynamic_experimental_control(events)

  def update_button_frame_counts(self, CS) -> None:
    for button in self.button_frame_counts:
      if self.button_frame_counts[button] > 0:
        self.button_frame_counts[button] += 1

    for button_event in CS.buttonEvents:
      button = button_event.type.raw
      if button in self.button_frame_counts:
        self.button_frame_counts[button] = int(button_event.pressed)

  def update_dynamic_experimental_control(self, events) -> None:
    # DEC and the planner read the param back within a second, so the switch applies while driving
    global last_dec_switch
    if self.button_frame_counts[ButtonType.gapAdjustCruise] >= DISTANCE_LONG_PRESS and not self.experimental_mode_switched:
      self._dynamic_experimental_control = not self.params.get_bool("DynamicExperimentalControl")
      self.params.put_bool("DynamicExperimentalControl", self._dynamic_experimental_control)
      last_dec_switch = self._dynamic_experimental_control
      events.add(EventNameSP.experimentalModeSwitched)
      self.experimental_mode_switched = True
