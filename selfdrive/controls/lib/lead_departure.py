from collections import deque
import math


class LeadDeparture:
  """Release a stop for a sustained radar lead departure, using the existing MPC acceleration."""
  def __init__(self):
    self.samples = deque()
    self.reset()

  def reset(self):
    self.samples.clear()
    self.last_t = None
    self.last_distance = None
    self.releasing = False

  def update(self, CS, radar, model_time, active, a_target):
    lead = radar.leadOne
    radar_time = radar.mdMonoTime * 1e-9
    if (not active or not CS.canValid or CS.gasPressed or CS.brakePressed or CS.cruiseState.standstill or CS.vEgo >= 0.5 or
        not lead.status or not lead.radar or not all(math.isfinite(x) for x in (CS.vEgo, lead.dRel, lead.vLead, lead.vRel, a_target)) or
        lead.dRel <= 0 or lead.vLead < 0.2 or lead.vRel < 0.2 or
        any(radar.radarErrors.to_dict().values()) or radar_time <= 0 or not 0 <= model_time - radar_time <= 0.25):
      self.reset()
      return False

    if self.last_t is not None and radar_time < self.last_t:
      self.reset()

    if radar_time != self.last_t:
      if self.last_t is not None:
        elapsed = radar_time - self.last_t
        # Radar track IDs can alternate on one vehicle; check spatial continuity instead.
        if elapsed > 0.25 or abs(lead.dRel - self.last_distance) > lead.vRel * elapsed + 0.5:
          self.reset()
      self.last_t, self.last_distance = radar_time, lead.dRel
      if not self.samples and not CS.standstill:
        return False
      self.samples.append((radar_time, lead.dRel))
      while len(self.samples) > 1 and radar_time - self.samples[1][0] >= 0.4:
        self.samples.popleft()

    # Require 0.4 s of motion, an opening gap, and more than slow creep.
    confirmed = bool(self.samples and radar_time - self.samples[0][0] >= 0.4 and
                     lead.dRel - self.samples[0][1] >= 0.2 and lead.vLead >= 0.5)
    # Hysteresis avoids reapplying the stop for small positive target fluctuations.
    # A stopped/lost lead or a nonpositive MPC target immediately restores normal stopping.
    self.releasing = (self.releasing or confirmed) and a_target > (0.0 if self.releasing else 0.05)
    return self.releasing
