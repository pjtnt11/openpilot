import pytest
from cereal import car, log

from openpilot.selfdrive.controls.lib.lead_departure import LeadDeparture


class TestLeadDeparture:
  def setup_method(self):
    self.departure = LeadDeparture()
    self.cs = car.CarState.new_message(canValid=True, standstill=True)
    self.radar = log.RadarState.new_message()
    self.radar.leadOne = {'status': True, 'radar': True, 'dRel': 6., 'vLead': 0.6, 'vRel': 0.6}
    self.t = 10.

  def update(self, accel=0.08, active=True, dt=0.05, distance_step=0.03):
    self.t += dt
    self.radar.mdMonoTime = round(self.t * 1e9)
    self.radar.leadOne.dRel += distance_step
    return self.departure.update(self.cs, self.radar, self.t, active, accel)

  def confirm(self):
    for _ in range(12):
      result = self.update()
    assert result

  def test_requires_sustained_motion_and_an_opening_gap(self):
    for _ in range(8):
      assert not self.update()
    self.confirm()

  @pytest.mark.parametrize('speed', [0., 0.1, 0.3])
  def test_stationary_or_slowly_creeping_lead_does_not_release(self, speed):
    self.radar.leadOne.vLead = self.radar.leadOne.vRel = speed
    for _ in range(40):
      assert not self.update(distance_step=speed * 0.05)

  def test_speed_noise_without_gap_growth_does_not_release(self):
    for i in range(40):
      assert not self.update(distance_step=0.04 if i % 2 else -0.04)

  def test_brief_departure_then_stop_does_not_release(self):
    for _ in range(5):
      assert not self.update()
    self.radar.leadOne.vLead = self.radar.leadOne.vRel = 0.
    assert not self.update()
    self.radar.leadOne.vLead = self.radar.leadOne.vRel = 0.6
    for _ in range(8):
      assert not self.update()
    self.confirm()

  def test_small_positive_targets_do_not_reapply_stop_after_confirmation(self):
    self.confirm()
    for target in (0.09, 0.03, 0.08, 0.02):
      assert self.update(accel=target)
    assert not self.update(accel=0.)
    assert not self.update(accel=0.03)
    assert self.update(accel=0.08)
    assert not self.update(accel=-0.1)

  def test_needs_positive_planned_acceleration_to_begin(self):
    for _ in range(20):
      assert not self.update(accel=0.04)
    assert self.update(accel=0.08)

  @pytest.mark.parametrize('target', [float('nan'), float('inf'), -float('inf')])
  def test_invalid_planned_acceleration_clears_confirmation(self, target):
    self.confirm()
    assert not self.update(accel=target)

  @pytest.mark.parametrize('condition', ['gas', 'brake', 'invalid_can', 'pcm_hold', 'moving', 'inactive'])
  def test_invalid_vehicle_state_clears_confirmation(self, condition):
    self.confirm()
    if condition == 'gas':
      self.cs.gasPressed = True
    elif condition == 'brake':
      self.cs.brakePressed = True
    elif condition == 'invalid_can':
      self.cs.canValid = False
    elif condition == 'pcm_hold':
      self.cs.cruiseState.standstill = True
    elif condition == 'moving':
      self.cs.vEgo = 0.5
    assert not self.update(active=condition != 'inactive')
    assert not self.departure.samples

  @pytest.mark.parametrize('condition', ['lost', 'vision_only', 'stopped', 'closing', 'nan_distance', 'nan_speed'])
  def test_invalid_lead_clears_confirmation(self, condition):
    self.confirm()
    lead = self.radar.leadOne
    if condition == 'lost':
      lead.status = False
    elif condition == 'vision_only':
      lead.radar = False
    elif condition == 'stopped':
      lead.vLead = 0.
    elif condition == 'closing':
      lead.vRel = -0.1
    elif condition == 'nan_distance':
      lead.dRel = float('nan')
    elif condition == 'nan_speed':
      lead.vLead = float('nan')
    assert not self.update()

  @pytest.mark.parametrize('error', ['canError', 'radarFault', 'wrongConfig', 'radarUnavailableTemporary'])
  def test_radar_error_clears_confirmation(self, error):
    self.confirm()
    setattr(self.radar.radarErrors, error, True)
    assert not self.update()

  def test_reused_radar_frame_cannot_confirm_departure(self):
    assert not self.update()
    for elapsed in (0., 0.05, 0.1, 0.2, 0.3, 0.5):
      assert not self.departure.update(self.cs, self.radar, self.t + elapsed, True, 0.08)

  def test_stale_radar_clears_existing_confirmation(self):
    self.confirm()
    assert not self.departure.update(self.cs, self.radar, self.t + 0.3, True, 0.08)

  @pytest.mark.parametrize('dt,distance_step', [(0.5, 0.3), (-0.1, 0.03), (0.05, 3.), (0.05, -3.)])
  def test_discontinuity_requires_new_confirmation(self, dt, distance_step):
    self.confirm()
    assert not self.update(dt=dt, distance_step=distance_step)
    self.confirm()

  def test_radar_track_id_changes_on_same_departing_car_are_tolerated(self):
    for i in range(12):
      # The recorded Corolla lead alternates radar tracks while its range stays continuous.
      self.radar.leadOne.radarTrackId = 2134 if i % 2 else 2144
      result = self.update()
    assert result

  def test_cannot_arm_during_ordinary_moving_following(self):
    self.cs.standstill = False
    self.cs.vEgo = 0.3
    for _ in range(20):
      assert not self.update()

  def test_confirmation_survives_initial_wheel_motion(self):
    self.confirm()
    self.cs.standstill = False
    self.cs.vEgo = 0.2
    assert self.update(accel=0.03)
    self.cs.vEgo = 0.5
    assert not self.update()


class PlannerMessages(dict):
  def __init__(self):
    super().__init__({
      'carState': car.CarState.new_message(canValid=True, standstill=True, vCruise=85.),
      'carControl': car.CarControl.new_message(),
      'controlsState': log.ControlsState.new_message(longControlState='stopping'),
      'selfdriveState': log.SelfdriveState.new_message(enabled=True, personality='standard'),
      'liveParameters': log.LiveParametersData.new_message(),
      'radarState': log.RadarState.new_message(),
      'modelV2': log.ModelDataV2.new_message(),
    })
    self.logMonoTime = {'modelV2': 0}


class TestPlannerLeadDeparture:
  @pytest.mark.parametrize('condition', ['normal', 'experimental_stop', 'force_decel', 'stock_longitudinal', 'second_lead'])
  def test_release_only_changes_stop_decision_for_confirmed_departure(self, monkeypatch, condition):
    # Uses the native MPC with the same inputs in both planners. Disabling only
    # departure recognition provides the previous stop policy for comparison.
    from opendbc.car.toyota.interface import CarInterface
    from opendbc.car.toyota.values import CAR
    from openpilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlanner

    cp = CarInterface.get_non_essential_params(CAR.TOYOTA_COROLLA_TSS2)
    cp.openpilotLongitudinalControl = condition != 'stock_longitudinal'
    before, after = LongitudinalPlanner(cp), LongitudinalPlanner(cp)
    monkeypatch.setattr(before.lead_departure, 'update', lambda *args: False)
    sm = PlannerMessages()
    sm['selfdriveState'].experimentalMode = condition == 'experimental_stop'
    sm['modelV2'].action.shouldStop = condition == 'experimental_stop'
    sm['controlsState'].forceDecel = condition == 'force_decel'
    distance = 3.5
    advances = 0
    for frame in range(100):
      speed = 0. if frame < 20 else 0.55
      distance += speed * 0.05
      sm.logMonoTime['modelV2'] = 10_000_000_000 + frame * 50_000_000
      sm['radarState'].mdMonoTime = sm.logMonoTime['modelV2']
      sm['radarState'].leadOne = {'status': True, 'radar': True, 'dRel': distance, 'vLead': speed, 'vRel': speed,
                                  'aLeadTau': 1.5, 'modelProb': 1.}
      if condition == 'second_lead':
        sm['radarState'].leadOne.dRel += 10.
        sm['radarState'].leadTwo = {'status': True, 'radar': True, 'dRel': 3.5, 'aLeadTau': 1.5, 'modelProb': 1.}
      before.update(sm)
      after.update(sm)
      assert after.output_a_target == pytest.approx(before.output_a_target, abs=1e-8)
      assert after.fcw == before.fcw
      if before.output_should_stop != after.output_should_stop:
        assert condition == 'normal'
        assert before.output_should_stop and not after.output_should_stop
        assert 0 < after.output_a_target < 0.1
        advances += 1
    assert (advances > 0) == (condition == 'normal')

  @pytest.mark.parametrize('gap', [3.5, 6.])
  @pytest.mark.parametrize('maneuver', ['depart', 'creep', 'depart_then_stop'])
  def test_closed_loop_departure_and_aborted_departure(self, monkeypatch, gap, maneuver):
    from opendbc.car.toyota.interface import CarInterface
    from opendbc.car.toyota.values import CAR
    from openpilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlanner

    def simulate(use_departure):
      planner = LongitudinalPlanner(CarInterface.get_non_essential_params(CAR.TOYOTA_COROLLA_TSS2))
      if not use_departure:
        monkeypatch.setattr(planner.lead_departure, 'update', lambda *args: False)
      sm = PlannerMessages()
      speed, accel, distance, lead_distance, previous_lead_speed = 0., 0., 0., gap, 0.
      first_release = None
      stop_after_release = False
      trajectory = []
      for frame in range(400):
        t = frame * 0.05
        lead_speed = min(max(t - 3., 0.), 5.)
        if maneuver == 'creep':
          lead_speed = min(lead_speed, 0.1)
        elif maneuver == 'depart_then_stop' and t >= 4.:
          lead_speed = max(1. - 2. * (t - 4.), 0.)
        lead_distance += lead_speed * 0.05
        sm['carState'].vEgo = speed
        sm['carState'].aEgo = accel
        sm['carState'].standstill = speed < 0.01
        sm.logMonoTime['modelV2'] = 10_000_000_000 + frame * 50_000_000
        sm['radarState'].mdMonoTime = sm.logMonoTime['modelV2']
        sm['radarState'].leadOne = {'status': True, 'radar': True, 'dRel': lead_distance - distance,
                                    'vLead': lead_speed, 'vRel': lead_speed - speed,
                                    'aLeadK': (lead_speed - previous_lead_speed) / 0.05, 'aLeadTau': 1.5, 'modelProb': 1.}
        previous_lead_speed = lead_speed
        planner.update(sm)
        if not planner.output_should_stop and first_release is None:
          first_release = t
        if planner.output_should_stop and first_release is not None:
          stop_after_release = True
        # Same ideal acceleration plant as the existing longitudinal maneuver tests.
        accel = float(min(-0.5, planner.output_a_target) if planner.output_should_stop else planner.output_a_target)
        speed = max(0., speed + accel * 0.05)
        if speed == 0.:
          accel = 0.
        distance += speed * 0.05
        sm['controlsState'].longControlState = 'stopping' if planner.output_should_stop else 'pid'
        assert lead_distance - distance >= gap - 0.1
        trajectory.append((speed, distance))
      return first_release, stop_after_release, trajectory

    before, after = simulate(False), simulate(True)
    if maneuver == 'depart':
      assert after[0] <= before[0]
      assert not after[1]  # No stop/release chatter during a continuing departure.
    elif maneuver == 'creep':
      assert after[2] == before[2]
    else:
      assert after[2][-1][0] < 0.05
