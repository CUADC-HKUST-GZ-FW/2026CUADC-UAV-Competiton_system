"""Project-local MAVLink partial mission adapter tests."""

import threading
import time
from types import SimpleNamespace

from pymavlink import mavutil

from uav_fcu_interface.mission_partial_adapter import (
    MissionPartialUpdateAdapter,
)


class FakeMissionMessage:
    def __init__(self, message_type, **fields):
        self._message_type = message_type
        self.__dict__.update(fields)

    def get_type(self):
        return self._message_type

    def get_srcSystem(self):
        return 1


def test_single_waypoint_partial_write_uses_inclusive_seq_range():
    adapter = MissionPartialUpdateAdapter(SimpleNamespace(publish=lambda _msg: None))
    outbound = []
    adapter._publish = outbound.append
    waypoint = SimpleNamespace(
        frame=3,
        command=16,
        autocontinue=True,
        param1=0.0,
        param2=8.0,
        param3=0.0,
        param4=0.0,
        x_lat=22.8812345,
        y_long=113.4912345,
        z_alt=35.0,
    )
    result = []
    worker = threading.Thread(
        target=lambda: result.append(
            adapter.push_one(7, waypoint, 1.0, lambda: None)
        )
    )
    worker.start()
    deadline = time.monotonic() + 1.0
    while not outbound and time.monotonic() < deadline:
        time.sleep(0.001)

    write_partial = outbound[0]
    assert write_partial.get_type() == 'MISSION_WRITE_PARTIAL_LIST'
    assert write_partial.start_index == 7
    assert write_partial.end_index == 7

    adapter.handle_mavlink_message(FakeMissionMessage(
        'MISSION_REQUEST_INT', seq=7,
    ))
    assert outbound[1].get_type() == 'MISSION_ITEM_INT'
    assert outbound[1].seq == 7
    adapter.handle_mavlink_message(FakeMissionMessage(
        'MISSION_ACK', type=mavutil.mavlink.MAV_MISSION_ACCEPTED,
    ))
    worker.join(timeout=1.0)

    assert not worker.is_alive()
    assert result == [1]
