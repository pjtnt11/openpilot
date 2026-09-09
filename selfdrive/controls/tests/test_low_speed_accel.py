import numpy as np
import pytest

from opendbc.car.interfaces import ACCEL_MAX, ACCEL_MIN
from opendbc.car.toyota.interface import CarInterface
from opendbc.car.toyota.values import CAR
from openpilot.common.constants import CV
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import T_IDXS, LongitudinalPlanSource
from openpilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlanner, get_human_max_accel, limit_accel_in_turns
from openpilot.selfdrive.controls.tests.test_lead_departure import PlannerMessages


@pytest.fixture
def cp():
  return CarInterface.get_non_essential_params(CAR.TOYOTA_COROLLA_TSS2)


@pytest.mark.parametrize('speed_mph', [0., 5., 15., 20.])
def test_full_low_speed_acceleration_available(cp, speed_mph):
  speed = speed_mph * CV.MPH_TO_MS
  cap = get_human_max_accel(speed, 85. / 3.6)
  assert cap == ACCEL_MAX
  assert limit_accel_in_turns(speed, 0., [ACCEL_MIN, cap], cp) == [ACCEL_MIN, ACCEL_MAX]


@pytest.mark.parametrize('speed_mph,requested_accel', [(7.2, 1.652), (14.9, 1.760), (20.1, 1.613), (24.9, 1.329), (34.2, 1.039)])
def test_recorded_departure_requests_fit_below_ceiling(cp, speed_mph, requested_accel):
  speed = speed_mph * CV.MPH_TO_MS
  cap = get_human_max_accel(speed, 85. / 3.6)
  assert limit_accel_in_turns(speed, 0., [ACCEL_MIN, cap], cp)[1] >= requested_accel


def test_high_speed_profile_is_unchanged(cp):
  # Freeze the previous curve, including the taper as the car approaches set speed.
  for speed in np.linspace(35. * CV.MPH_TO_MS, 40., 50):
    for delta in [-5., 0., 0.5, 1., 2., 4., 5., 20.]:
      cruise = speed + delta
      base = np.interp(speed, [0., 10., 25., 40.], [1.6, 1.2, 0.8, 0.6])
      set_speed_cap = np.interp(cruise, [0., 12.5, 25.], [base / 4, base / 2, base])
      approach_cap = np.interp(delta, [0., 1., 5.], [0., 0.5, base])
      expected = min(base, set_speed_cap, approach_cap)
      assert get_human_max_accel(speed, cruise) == pytest.approx(expected, abs=1e-12)
      for angle in [-90., 0., 90.]:
        total = np.interp(speed, [20., 40.], [1.7, 3.2])
        lateral = speed ** 2 * angle * CV.DEG_TO_RAD / (cp.steerRatio * cp.wheelbase)
        expected_turn = min(expected, np.sqrt(max(total ** 2 - lateral ** 2, 0.)))
        assert limit_accel_in_turns(speed, angle, [ACCEL_MIN, expected], cp) == pytest.approx([ACCEL_MIN, expected_turn])


def test_transition_is_continuous_and_falls_back_to_cruise_curve():
  speeds = np.linspace(19. * CV.MPH_TO_MS, 36. * CV.MPH_TO_MS, 1000)
  caps = np.array([get_human_max_accel(v, 85. / 3.6) for v in speeds])
  assert np.all(np.diff(caps) <= 0.)
  assert np.max(np.abs(np.diff(caps))) < .002
  for boundary in [20., 35.]:
    v = boundary * CV.MPH_TO_MS
    assert get_human_max_accel(v - 1e-6, 85. / 3.6) == pytest.approx(get_human_max_accel(v + 1e-6, 85. / 3.6), abs=1e-6)


@pytest.mark.parametrize('speed_mph', [0., 5., 15., 20., 30., 55.])
def test_set_speed_approach_still_limits_acceleration(speed_mph):
  speed = speed_mph * CV.MPH_TO_MS
  assert get_human_max_accel(speed, speed) == 0.
  assert get_human_max_accel(speed, speed - 1.) == 0.
  assert get_human_max_accel(speed, speed + 1.) <= .5


def test_cornering_still_reduces_acceleration(cp):
  speed = 20. * CV.MPH_TO_MS
  ceilings = [limit_accel_in_turns(speed, angle, [ACCEL_MIN, ACCEL_MAX], cp)[1] for angle in [0., 15., 30., 45., 60.]]
  assert all(a > b for a, b in zip(ceilings, ceilings[1:], strict=False))
  assert ceilings[-1] == 0.
  assert limit_accel_in_turns(speed, -30., [ACCEL_MIN, ACCEL_MAX], cp)[1] == ceilings[2]


@pytest.mark.parametrize('condition', ['accelerate', 'brake', 'experimental', 'throttle_restricted', 'near_set_speed'])
def test_planner_output_preserves_other_constraints(cp, monkeypatch, condition):
  speed = 15. * CV.MPH_TO_MS
  request = -0.8 if condition == 'brake' else 1.8
  planner = LongitudinalPlanner(cp, init_v=speed)
  sm = PlannerMessages()
  sm['carState'].vEgo = speed
  sm['carState'].standstill = False
  sm['carControl'].orientationNED = [0., 0., 0.]
  sm['controlsState'].longControlState = 'pid'
  sm['selfdriveState'].experimentalMode = condition == 'experimental'
  sm['modelV2'].action.desiredAcceleration = .4
  sm['modelV2'].action.shouldStop = condition == 'experimental'
  sm['modelV2'].meta.disengagePredictions.gasPressProbs = [0. if condition == 'throttle_restricted' else 1.] * 6
  if condition == 'near_set_speed':
    sm['carState'].vCruise = (speed + 1.) * 3.6

  def planned_acceleration(*args, **kwargs):
    planner.mpc.v_solution = speed + request * T_IDXS
    planner.mpc.a_solution[:] = request
    planner.mpc.j_solution[:] = 0.
    planner.mpc.source = LongitudinalPlanSource.cruise

  monkeypatch.setattr(planner.mpc, 'update', planned_acceleration)
  # Let the existing ceiling slew limiter settle, including a throttle restriction.
  for _ in range(60):
    planner.update(sm)

  expected = {'accelerate': 1.8, 'brake': -.8, 'experimental': .4, 'throttle_restricted': -.3, 'near_set_speed': .5}[condition]
  assert planner.output_a_target == pytest.approx(expected, abs=1e-5)
  assert planner.output_should_stop == (condition == 'experimental')
