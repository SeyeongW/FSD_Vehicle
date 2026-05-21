import json

import pytest

from waver_patrol.comms.process_lock import ProcessLock
from waver_patrol.comms.serial_json_client import FakeSerial, SerialJsonClient
from waver_patrol.safety.command import WheelCommand


def test_fake_serial_command_and_stop():
    fake = FakeSerial()
    client = SerialJsonClient(transport=fake)
    client.send_command(WheelCommand(0.1, -0.1, source="manual"))
    client.send_stop(repeat=2, delay_s=0.0)
    assert json.loads(fake.lines[0]) == {"T": 1, "L": 0.1, "R": -0.1}
    assert json.loads(fake.lines[-1]) == {"T": 1, "L": 0, "R": 0}


def test_fake_serial_disconnect_raises():
    client = SerialJsonClient(transport=FakeSerial(fail_after=0))
    with pytest.raises(OSError):
        client.send_command(WheelCommand(0.1, 0.1, source="manual"))


def test_process_lock_duplicate(tmp_path):
    first = ProcessLock(tmp_path / "waver.lock")
    second = ProcessLock(tmp_path / "waver.lock")
    assert first.acquire()
    assert not second.acquire()
    first.release()
