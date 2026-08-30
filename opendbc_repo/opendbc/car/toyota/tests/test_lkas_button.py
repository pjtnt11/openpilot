import pytest

from opendbc.can import CANPacker
from opendbc.car import Bus
from opendbc.car.toyota.carstate import CarState
from opendbc.car.toyota.interface import CarInterface
from opendbc.car.toyota.values import CAR, DBC


PRESS = [("lkas", True), ("lkas", False)]


class TestLkasButton:
  def setup_method(self):
    self.setup_car(CAR.TOYOTA_COROLLA_TSS2)

  def setup_car(self, car):
    self.cs = CarState(CarInterface.get_non_essential_params(car))
    self.parsers = self.cs.get_can_parsers(self.cs.CP)
    self.packer = CANPacker(DBC[car][Bus.pt])
    self.now = 0
    assert self.update() == []

  def hud(self, status, notification=0, bus=2, unavailable=0):
    return self.packer.make_can_msg("LKAS_HUD", bus, {
      "LKAS_STATUS": status, "LDA_ON_MESSAGE": notification, "LDA_SA_TOGGLE": 1,
      "LDA_UNAVAILABLE": unavailable,
    })

  def update(self, *frames):
    packets = []
    for frame in frames:
      self.now += 1_000_000_000
      packets.append((self.now, [frame]))
    for parser in self.parsers.values():
      parser.update(packets)
    return [(str(b.type), b.pressed) for b in self.cs.update(self.parsers).buttonEvents if str(b.type) == "lkas"]

  def test_counted_ten_taps(self):
    # Camera-bus payloads from the counted Corolla test: first tap turns LDA off
    # while LDA_ON_MESSAGE is already zero. Each tap must produce one pair.
    assert self.update((0x412, bytes.fromhex("68040c00101c0812"), 2)) == []
    for payload in ["00040c00101c3812", "68040c40101c3812"] * 5:
      assert self.update((0x412, bytes.fromhex(payload), 2)) == PRESS

  @pytest.mark.parametrize("status,unavailable", [(0, 0), (1, 0), (2, 0), (0, 1)])
  def test_first_received_state_is_not_a_press(self, status, unavailable):
    assert self.update(self.hud(status, notification=1, unavailable=unavailable)) == []

  def test_becoming_unavailable_is_not_a_press(self):
    # Recorded Corolla sequence: LDA becomes unavailable without being switched off.
    for payload in ("68040c00101c0812", "95040c00101c0812", "28140d00101c3812"):
      assert self.update((0x412, bytes.fromhex(payload), 2)) == []

  def test_tap_from_unavailable_emits_press(self):
    # The missed first taps only clear LDA_UNAVAILABLE; LKAS_STATUS remains zero.
    assert self.update((0x412, bytes.fromhex("28140d00101c3812"), 2)) == []
    assert self.update((0x412, bytes.fromhex("00040c00101c3812"), 2)) == PRESS
    assert self.update((0x412, bytes.fromhex("68040c40101c0812"), 2)) == PRESS

  def test_availability_changes_in_a_batch_are_not_presses(self):
    assert self.update(self.hud(1)) == []
    assert self.update(self.hud(2), self.hud(0, unavailable=1), self.hud(1)) == []

  def test_notification_and_nonzero_status_changes_are_not_presses(self):
    assert self.update(self.hud(1)) == []
    for status, notification in [(1, 1), (1, 0), (2, 0), (1, 2), (1, 0)]:
      assert self.update(self.hud(status, notification)) == []

  def test_each_transition_in_a_batch_is_emitted_once(self):
    assert self.update(self.hud(1)) == []
    assert self.update(self.hud(0), self.hud(1, notification=1)) == PRESS * 2
    assert self.update() == []
    assert self.update(self.hud(1, notification=1)) == []

  def test_only_camera_bus_initializes_and_triggers_button(self):
    assert self.update(self.hud(0, bus=0)) == []
    assert self.update(self.hud(1)) == []
    assert self.update(self.hud(0, bus=0)) == []
    assert self.update(self.hud(0)) == PRESS

  def test_other_tss2_retains_notification_events(self):
    self.setup_car(CAR.TOYOTA_RAV4_TSS2)
    assert self.update(self.hud(1, notification=1)) == PRESS
    assert self.update(self.hud(1, notification=2)) == PRESS
    assert self.update(self.hud(0)) == []
