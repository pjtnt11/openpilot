from types import SimpleNamespace

import pytest
from cereal import car, log
from openpilot.common.params import Params
from openpilot.selfdrive.selfdrived.alertmanager import AlertManager
from openpilot.selfdrive.selfdrived.events import EmptyAlert, Events, ET
from openpilot.selfdrive.selfdrived.selfdrived import SelfdriveD


def car_state(buttons=None, **kwargs):
  cs = car.CarState.new_message(canValid=True, **kwargs)
  cs.buttonEvents = buttons if buttons is not None else [
    {'type': 'lkas', 'pressed': True}, {'type': 'lkas', 'pressed': False},
  ]
  return cs


class TestLkasExperimentalMode:
  def setup_method(self):
    # Exercise the real receive/toggle path without starting managed processes.
    self.sd = SelfdriveD.__new__(SelfdriveD)
    self.sd.CP = car.CarParams.new_message(openpilotLongitudinalControl=True)
    self.sd.params = Params()
    self.sd.params.put_bool('ExperimentalMode', False, block=True)
    self.sd.AM = AlertManager()
    self.sd.sm = SimpleNamespace(frame=10, update=lambda timeout: None)
    self.sd.car_state_sock = object()
    self.sd.CS_prev = car_state(buttons=[])
    self.sd.initialized = True
    self.sd.enabled = False

  def alert(self):
    self.sd.AM.process_alerts(self.sd.sm.frame, {ET.WARNING})
    return self.sd.AM.current_alert

  def test_taps_toggle_on_and_off_with_matching_alert(self):
    for expected in (True, False):
      self.sd.update_experimental_mode(car_state())
      assert self.sd.params.get_bool('ExperimentalMode') == expected
      assert self.alert().alert_text_1 == f"Experimental Mode {'On' if expected else 'Off'}"
      assert self.alert().alert_type == 'experimentalMode/permanent'
      assert self.alert().audible_alert == car.CarControl.HUDControl.AudibleAlert.none
      self.sd.sm.frame += 10

  @pytest.mark.parametrize('left,right', [(True, False), (False, True), (True, True)])
  def test_either_blinker_preserves_mode_and_button_events(self, left, right):
    cs = car_state(leftBlinker=left, rightBlinker=right)
    before = cs.to_dict()
    self.sd.update_experimental_mode(cs)
    assert not self.sd.params.get_bool('ExperimentalMode')
    assert self.alert() == EmptyAlert
    assert cs.to_dict() == before

  @pytest.mark.parametrize('buttons', [[], [{'type': 'lkas', 'pressed': False}], [{'type': 'gapAdjustCruise', 'pressed': True}]])
  def test_only_lkas_presses_toggle(self, buttons):
    self.sd.update_experimental_mode(car_state(buttons=buttons))
    assert not self.sd.params.get_bool('ExperimentalMode')
    assert self.alert() == EmptyAlert

  @pytest.mark.parametrize('condition', ['invalid_can', 'stock_longitudinal', 'passive'])
  def test_unsupported_or_invalid_state_does_not_toggle(self, condition):
    cs = car_state()
    if condition == 'invalid_can':
      cs.canValid = False
    elif condition == 'stock_longitudinal':
      self.sd.CP.openpilotLongitudinalControl = False
    else:
      self.sd.CP.passive = True
    self.sd.update_experimental_mode(cs)
    assert not self.sd.params.get_bool('ExperimentalMode')
    assert self.alert() == EmptyAlert

  def test_toggle_reads_current_setting(self):
    self.sd.params.put_bool('ExperimentalMode', True, block=True)
    self.sd.experimental_mode = False  # params thread can lag a UI change
    self.sd.update_experimental_mode(car_state())
    assert not self.sd.params.get_bool('ExperimentalMode')
    assert self.alert().alert_text_1 == 'Experimental Mode Off'

  def test_each_press_in_a_batch_toggles_once(self):
    cs = car_state()
    cs.buttonEvents = [be.to_dict() for be in cs.buttonEvents] * 2
    self.sd.update_experimental_mode(cs)
    assert not self.sd.params.get_bool('ExperimentalMode')
    assert self.alert().alert_text_1 == 'Experimental Mode Off'

  def test_missing_carstate_does_not_reuse_button_event(self, monkeypatch):
    packet = SimpleNamespace(carState=car_state())
    packets = iter([packet, None, None])
    monkeypatch.setattr('openpilot.selfdrive.selfdrived.selfdrived.messaging.recv_one', lambda sock: next(packets))
    self.sd.CS_prev = self.sd.data_sample()
    assert self.sd.params.get_bool('ExperimentalMode')
    for _ in range(2):
      self.sd.sm.frame += 1
      self.sd.data_sample()
      assert self.sd.params.get_bool('ExperimentalMode')
    assert self.sd.AM.alerts['experimentalMode/permanent'].added_frame == 10

  def test_notification_expires_and_yields_to_safety_alerts(self):
    self.sd.update_experimental_mode(car_state())
    duration = self.alert().duration
    events = Events()
    events.add(log.OnroadEvent.EventName.fcw)
    self.sd.AM.add_many(self.sd.sm.frame, events.create_alerts([ET.PERMANENT]))
    assert self.alert().alert_type == 'fcw/permanent'
    self.sd.sm.frame += duration + 1
    assert self.alert() == EmptyAlert
