from cereal import car, log

from openpilot.selfdrive.controls.lib.desire_helper import (
  DesireHelper, LANE_CHANGE_BUTTON_TIMEOUT, LANE_CHANGE_SPEED_MIN, LaneChangeButtonLatch,
)


def get_car_state(**kwargs):
  car_state = car.CarState.new_message()
  car_state.vEgo = LANE_CHANGE_SPEED_MIN + 1.0
  for key, value in kwargs.items():
    setattr(car_state, key, value)
  return car_state


class TestDesireHelper:
  @staticmethod
  def enter_pre_lane_change(helper, car_state):
    helper.update(car_state, lateral_active=True, lane_change_prob=1.0)
    assert helper.lane_change_state == log.LaneChangeState.preLaneChange

  def test_lane_change_button_starts_lane_change(self):
    helper = DesireHelper()
    car_state = get_car_state(leftBlinker=True)
    self.enter_pre_lane_change(helper, car_state)

    helper.update(car_state, lateral_active=True, lane_change_prob=1.0, lane_change_button_pressed=True)

    assert helper.lane_change_state == log.LaneChangeState.laneChangeStarting
    assert helper.lane_change_direction == log.LaneChangeDirection.left

  def test_lane_change_button_with_new_blinker_starts_lane_change(self):
    helper = DesireHelper()
    car_state = get_car_state(leftBlinker=True)

    helper.update(car_state, lateral_active=True, lane_change_prob=1.0, lane_change_button_pressed=True)

    assert helper.lane_change_state == log.LaneChangeState.laneChangeStarting

  def test_lane_change_button_respects_blindspot(self):
    helper = DesireHelper()
    car_state = get_car_state(rightBlinker=True, rightBlindspot=True)
    self.enter_pre_lane_change(helper, car_state)

    helper.update(car_state, lateral_active=True, lane_change_prob=1.0, lane_change_button_pressed=True)

    assert helper.lane_change_state == log.LaneChangeState.preLaneChange

  def test_steering_torque_still_starts_lane_change(self):
    helper = DesireHelper()
    car_state = get_car_state(rightBlinker=True, steeringPressed=True, steeringTorque=-1.0)

    helper.update(car_state, lateral_active=True, lane_change_prob=1.0)

    assert helper.lane_change_state == log.LaneChangeState.laneChangeStarting


class TestLaneChangeButtonLatch:
  def test_press_survives_until_consumed(self):
    latch = LaneChangeButtonLatch()
    car_state = get_car_state(leftBlinker=True)

    assert latch.update(car_state, True, log.LaneChangeState.preLaneChange, True, now=1.0)
    assert latch.update(car_state, True, log.LaneChangeState.preLaneChange, False, now=1.1)

    latch.consume()
    assert not latch.update(car_state, True, log.LaneChangeState.preLaneChange, False, now=1.2)

  def test_press_expires(self):
    latch = LaneChangeButtonLatch()
    car_state = get_car_state(rightBlinker=True)

    assert latch.update(car_state, True, log.LaneChangeState.preLaneChange, True, now=1.0)
    assert not latch.update(car_state, True, log.LaneChangeState.preLaneChange, False,
                            now=1.0 + LANE_CHANGE_BUTTON_TIMEOUT)

  def test_blindspot_clears_press(self):
    latch = LaneChangeButtonLatch()
    car_state = get_car_state(leftBlinker=True)

    assert latch.update(car_state, True, log.LaneChangeState.preLaneChange, True, now=1.0)
    car_state.leftBlindspot = True
    assert not latch.update(car_state, True, log.LaneChangeState.preLaneChange, False, now=1.1)
    car_state.leftBlindspot = False
    assert not latch.update(car_state, True, log.LaneChangeState.preLaneChange, False, now=1.2)

  def test_invalid_conditions_do_not_latch_press(self):
    latch = LaneChangeButtonLatch()

    assert not latch.update(get_car_state(), True, log.LaneChangeState.preLaneChange, True, now=1.0)
    assert not latch.update(get_car_state(leftBlinker=True), False,
                            log.LaneChangeState.preLaneChange, True, now=1.0)
    assert not latch.update(get_car_state(leftBlinker=True, vEgo=LANE_CHANGE_SPEED_MIN - 1.), True,
                            log.LaneChangeState.preLaneChange, True, now=1.0)
    assert not latch.update(get_car_state(leftBlinker=True), True,
                            log.LaneChangeState.laneChangeStarting, True, now=1.0)

  def test_direction_change_clears_pending_press(self):
    latch = LaneChangeButtonLatch()

    assert latch.update(get_car_state(leftBlinker=True), True,
                        log.LaneChangeState.preLaneChange, True, now=1.0)
    assert not latch.update(get_car_state(rightBlinker=True), True,
                            log.LaneChangeState.preLaneChange, False, now=1.1)
