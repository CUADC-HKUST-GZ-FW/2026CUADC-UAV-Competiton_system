"""Single-item MAVLink mission partial-update adapter.

MAVROS 2.14.0 passes its exclusive internal end index directly to the
inclusive MAVLink MISSION_WRITE_PARTIAL_LIST.end_index field.  This adapter
uses MAVROS' raw MAVLink bridge but owns the small partial-write transaction,
so no installed MAVROS file needs to be patched.
"""

import threading
import time

from mavros.mavlink import convert_to_bytes, convert_to_rosmsg
from pymavlink import mavutil


class PartialUpdateError(RuntimeError):
    """Raised when the FCU does not complete a single-item partial update."""


class PartialUpdateAborted(PartialUpdateError):
    """Raised when the owner deadline/cancellation guard stops the transaction."""


class MissionPartialUpdateAdapter:
    """Perform exactly one inclusive [seq, seq] MAVLink mission write."""

    def __init__(
        self,
        publisher,
        target_system=1,
        target_component=1,
        source_system=255,
        source_component=190,
        message_observer=None,
    ):
        self._publisher = publisher
        self._target_system = int(target_system)
        self._target_component = int(target_component)
        self._encoder = mavutil.mavlink.MAVLink(
            None,
            srcSystem=int(source_system),
            srcComponent=int(source_component),
        )
        self._parser = mavutil.mavlink.MAVLink(None)
        self._condition = threading.Condition()
        self._parser_lock = threading.Lock()
        self._transaction = None
        self._message_observer = message_observer

    def handle_ros_message(self, ros_message):
        """Decode a MAVROS passthrough message and advance an active transaction."""
        try:
            with self._parser_lock:
                messages = self._parser.parse_buffer(
                    bytes(convert_to_bytes(ros_message))
                ) or []
        except Exception:  # malformed/unrelated passthrough data is non-fatal
            return
        for message in messages:
            self.handle_mavlink_message(message)
            if self._message_observer is not None:
                self._message_observer(message)

    def handle_mavlink_message(self, message):
        """Handle request/ACK messages; public for deterministic unit tests."""
        message_type = str(message.get_type())
        outbound = None
        with self._condition:
            transaction = self._transaction
            if transaction is None:
                return
            if (
                hasattr(message, 'get_srcSystem')
                and int(message.get_srcSystem()) != self._target_system
            ):
                return

            if message_type in {'MISSION_REQUEST', 'MISSION_REQUEST_INT'}:
                requested_seq = int(message.seq)
                if requested_seq != transaction['seq']:
                    transaction['error'] = (
                        f'FCU requested unexpected partial seq={requested_seq}; '
                        f'expected={transaction["seq"]}'
                    )
                    self._condition.notify_all()
                    return
                outbound = self._encode_waypoint(
                    transaction['seq'],
                    transaction['waypoint'],
                    use_int=message_type == 'MISSION_REQUEST_INT',
                )
                transaction['item_sent'] = True
            elif message_type == 'MISSION_ACK':
                if int(message.type) == int(mavutil.mavlink.MAV_MISSION_ACCEPTED):
                    if transaction['item_sent']:
                        transaction['success'] = True
                else:
                    transaction['error'] = (
                        f'MISSION_ACK type={int(message.type)}'
                    )
                self._condition.notify_all()

        if outbound is not None:
            self._publish(outbound)

    def push_one(self, seq, waypoint, timeout_sec, abort_check):
        """Block until one item is ACKed, rejected, aborted, or timed out."""
        seq = int(seq)
        deadline = time.monotonic() + max(0.0, float(timeout_sec))
        with self._condition:
            if self._transaction is not None:
                raise PartialUpdateError('partial update adapter is busy')
            self._transaction = {
                'seq': seq,
                'waypoint': waypoint,
                'item_sent': False,
                'success': False,
                'error': None,
            }

        try:
            # MAVLink end_index is inclusive. A single R update is [seq, seq].
            self._publish(
                self._encoder.mission_write_partial_list_encode(
                    self._target_system,
                    self._target_component,
                    seq,
                    seq,
                )
            )
            while True:
                abort_reason = abort_check()
                if abort_reason:
                    raise PartialUpdateAborted(str(abort_reason))
                with self._condition:
                    transaction = self._transaction
                    if transaction['success']:
                        return 1
                    if transaction['error']:
                        raise PartialUpdateError(transaction['error'])
                    remaining = deadline - time.monotonic()
                    if remaining <= 0.0:
                        raise TimeoutError('single-item partial update timed out')
                    self._condition.wait(timeout=min(0.05, remaining))
        finally:
            with self._condition:
                self._transaction = None
                self._condition.notify_all()

    def _encode_waypoint(self, seq, waypoint, use_int):
        common = (
            self._target_system,
            self._target_component,
            int(seq),
            int(waypoint.frame),
            int(waypoint.command),
            0,
            int(bool(waypoint.autocontinue)),
            float(waypoint.param1),
            float(waypoint.param2),
            float(waypoint.param3),
            float(waypoint.param4),
        )
        if use_int:
            return self._encoder.mission_item_int_encode(
                *common,
                int(round(float(waypoint.x_lat) * 10_000_000.0)),
                int(round(float(waypoint.y_long) * 10_000_000.0)),
                float(waypoint.z_alt),
            )
        return self._encoder.mission_item_encode(
            *common,
            float(waypoint.x_lat),
            float(waypoint.y_long),
            float(waypoint.z_alt),
        )

    def _publish(self, message):
        message.pack(self._encoder)
        self._publisher.publish(convert_to_rosmsg(message))
