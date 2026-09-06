"""只读载荷监控状态机，不依赖ROS，也不包含任何控制接口。"""

from dataclasses import dataclass


MAV_CMD_DO_SET_SERVO = 183


class PayloadMonitorState:
    IDLE = 'IDLE'
    WAITING_FOR_COMMAND_UPLOAD = 'WAITING_FOR_COMMAND_UPLOAD'
    COMMAND_UPLOADED = 'COMMAND_UPLOADED'
    WAITING_FOR_COMMAND_REACHED = 'WAITING_FOR_COMMAND_REACHED'
    COMMAND_REACHED = 'COMMAND_REACHED'
    WAITING_FOR_PWM_CONFIRMATION = 'WAITING_FOR_PWM_CONFIRMATION'
    PWM_CONFIRMED = 'PWM_CONFIRMED'
    TIMEOUT = 'TIMEOUT'
    INVALID = 'INVALID'


@dataclass(frozen=True)
class PayloadMonitorConfig:
    servo_channel: int = 7
    release_pwm: int = 1900
    pwm_tolerance_us: int = 20
    required_consecutive_samples: int = 3
    telemetry_stale_timeout_s: float = 1.0
    execution_timeout_s: float = 5.0
    warning_throttle_s: float = 5.0

    def validate(self):
        errors = []
        if self.servo_channel < 1:
            errors.append('servo_channel must be >= 1')
        if not 800 <= self.release_pwm <= 2200:
            errors.append('release_pwm must be within [800, 2200]')
        if self.pwm_tolerance_us <= 0:
            errors.append('pwm_tolerance_us must be > 0')
        if self.required_consecutive_samples < 1:
            errors.append('required_consecutive_samples must be >= 1')
        if self.telemetry_stale_timeout_s <= 0:
            errors.append('telemetry_stale_timeout_s must be > 0')
        if self.execution_timeout_s <= 0:
            errors.append('execution_timeout_s must be > 0')
        if self.warning_throttle_s <= 0:
            errors.append('warning_throttle_s must be > 0')
        if errors:
            raise ValueError('; '.join(errors))


@dataclass(frozen=True)
class MonitorEvent:
    level: str
    state: str
    message: str
    key: str


class PayloadMonitor:
    """根据飞控只读遥测确认命令上传、经过及PWM输出。"""

    def __init__(self, config):
        config.validate()
        self.config = config
        self.task_id = 'unknown'
        self.mission_epoch = 0
        self.state = PayloadMonitorState.IDLE
        self.release_command_seq = None
        self.release_command_seen = False
        self.release_command_reached = False
        self.release_pwm_confirmed = False
        self.release_confirmed_latched = False
        self.execution_window_armed = False
        self.execution_window_started_at = None
        self.last_current_seq = None
        self.last_reached_seq = None
        self.last_rc_out_time = None
        self.observed_pwm = None
        self.consecutive_pwm_matches = 0
        self.monitor_start_time = None
        self.execution_start_time = None
        self._mission_signature = None
        self._saw_release_seq = False
        self._passage_inferred = False
        self._emitted = set()
        self._warning_times = {}

    def set_task_id(self, task_id, now):
        task_id = task_id.strip() or 'unknown'
        if task_id == self.task_id and self.state != PayloadMonitorState.IDLE:
            return []
        self.task_id = task_id
        if self.release_confirmed_latched:
            return self._once(
                'release_already_confirmed', 'info',
                'release already confirmed; monitor remains latched'
            )
        self._reset_for_new_task(now)
        return []

    def _reset_for_new_task(self, now):
        self.mission_epoch += 1
        self.state = PayloadMonitorState.WAITING_FOR_COMMAND_UPLOAD
        self.release_command_seq = None
        self.release_command_seen = False
        self.release_command_reached = False
        self.release_pwm_confirmed = False
        self.execution_window_armed = False
        self.execution_window_started_at = None
        self.last_current_seq = None
        self.last_reached_seq = None
        self.last_rc_out_time = None
        self.observed_pwm = None
        self.consecutive_pwm_matches = 0
        # 当前接口没有可靠的“开始上传”只读事件，因此不能从收到任务ID起误算上传超时。
        self.monitor_start_time = None
        self.execution_start_time = None
        self._mission_signature = None
        self._saw_release_seq = False
        self._passage_inferred = False
        self._emitted.clear()
        self._warning_times.clear()

    @staticmethod
    def _signature(waypoints):
        return tuple(
            (
                int(wp.command),
                round(float(wp.param1), 3),
                round(float(wp.param2), 3),
                round(float(wp.x_lat), 7),
                round(float(wp.y_long), 7),
            )
            for wp in waypoints
        )

    def _matches_release(self, waypoint):
        return (
            int(waypoint.command) == MAV_CMD_DO_SET_SERVO
            and abs(float(waypoint.param1) - self.config.servo_channel) <= 0.01
            and abs(float(waypoint.param2) - self.config.release_pwm) <= 0.5
        )

    def observe_mission(self, waypoints, current_seq, now):
        if self.state == PayloadMonitorState.IDLE or self.release_confirmed_latched:
            return []
        events = []
        matches = [index for index, wp in enumerate(waypoints) if self._matches_release(wp)]

        if len(matches) > 1:
            self.state = PayloadMonitorState.INVALID
            return self._once(
                'ambiguous_release_commands', 'error',
                f'ambiguous release commands count={len(matches)}'
            )

        if not matches:
            if self.state not in (PayloadMonitorState.IDLE, PayloadMonitorState.PWM_CONFIRMED):
                events.extend(self._throttled(
                    'release_command_not_found', now, 'warning',
                    'release command not found in FCU mission'
                ))
            return events

        signature = self._signature(waypoints)
        release_seq = matches[0]
        if signature != self._mission_signature:
            self.mission_epoch += 1
            self._mission_signature = signature
            self.release_command_seq = release_seq
            self.release_command_seen = True
            self.release_command_reached = False
            self.release_pwm_confirmed = False
            self.last_current_seq = None
            self.consecutive_pwm_matches = 0
            self.monitor_start_time = now
            self.execution_start_time = None
            self.execution_window_armed = False
            self.execution_window_started_at = None
            self._saw_release_seq = False
            self._passage_inferred = False
            self._emitted.clear()
            self.state = PayloadMonitorState.COMMAND_UPLOADED
            events.extend(self._once(
                'command_uploaded', 'info',
                f'command uploaded seq={release_seq} '
                f'channel={self.config.servo_channel} pwm={self.config.release_pwm}'
            ))
            self.state = PayloadMonitorState.WAITING_FOR_COMMAND_REACHED

        events.extend(self.observe_current_seq(current_seq, now))
        return events

    def observe_current_seq(self, current_seq, now):
        if self.release_confirmed_latched:
            self.last_current_seq = int(current_seq)
            return []
        if self.release_command_seq is None or self.release_command_reached:
            self.last_current_seq = current_seq
            return []

        current_seq = int(current_seq)
        events = []
        if current_seq == self.release_command_seq:
            self._saw_release_seq = True
        elif current_seq > self.release_command_seq:
            if self._saw_release_seq or self.last_current_seq == self.release_command_seq:
                events.extend(self._mark_reached('current_seq_advanced', now))
                return events
            self._passage_inferred = True
            self.last_current_seq = current_seq
            events.extend(self._throttled(
                'command_passage_inferred', now, 'warning',
                'command passage inferred but not confirmed '
                f'seq={self.release_command_seq} current_seq={current_seq}'
            ))
            return events

        self.last_current_seq = current_seq
        return events

    def observe_waypoint_reached(self, reached_seq, now):
        self.last_reached_seq = int(reached_seq)
        if self.release_confirmed_latched:
            return []
        events = []
        if (
            self.release_command_seq is not None
            and not self.execution_window_armed
            and self.last_reached_seq
            >= max(0, self.release_command_seq - 1)
        ):
            self.execution_window_armed = True
            self.execution_window_started_at = now
            events.extend(self._once(
                'execution_window_armed', 'info',
                'execution confirmation window armed '
                f'release_seq={self.release_command_seq} '
                f'reached_seq={self.last_reached_seq} '
                'reason=pre_release_waypoint_reached'
            ))
        if (
            self.release_command_seq is not None
            and not self.release_command_reached
            and self.last_reached_seq == self.release_command_seq
        ):
            events.extend(self._mark_reached('waypoint_reached', now))
        return events

    def _mark_reached(self, evidence, now):
        self.release_command_reached = True
        self.state = PayloadMonitorState.COMMAND_REACHED
        events = self._once(
            'command_reached', 'info',
            f'command reached seq={self.release_command_seq} evidence={evidence}'
        )
        self.state = PayloadMonitorState.WAITING_FOR_PWM_CONFIRMATION
        self.execution_start_time = now
        self.last_current_seq = self.release_command_seq
        return events

    def observe_rc_out(self, channels, now):
        self.last_rc_out_time = now
        channel_index = self.config.servo_channel - 1
        if len(channels) <= channel_index:
            return self._throttled(
                'channel_unavailable', now, 'warning',
                f'channel unavailable channel={self.config.servo_channel} '
                f'received_channels={len(channels)}'
            )

        self.observed_pwm = int(channels[channel_index])
        if self.release_confirmed_latched:
            return []
        matched = abs(self.observed_pwm - self.config.release_pwm) <= self.config.pwm_tolerance_us

        if self._passage_inferred and not self.release_command_reached and matched:
            reached_events = self._mark_reached('seq_and_pwm', now)
        else:
            reached_events = []

        if self.state != PayloadMonitorState.WAITING_FOR_PWM_CONFIRMATION:
            return reached_events

        if matched:
            self.consecutive_pwm_matches += 1
        else:
            self.consecutive_pwm_matches = 0

        if self.consecutive_pwm_matches >= self.config.required_consecutive_samples:
            self.release_pwm_confirmed = True
            self.release_confirmed_latched = True
            self.state = PayloadMonitorState.PWM_CONFIRMED
            reached_events.extend(self._once(
                'pwm_confirmed', 'info',
                f'channel {self.config.servo_channel} PWM confirmed '
                f'expected={self.config.release_pwm} observed={self.observed_pwm} '
                f'samples={self.consecutive_pwm_matches}'
            ))
        return reached_events

    def tick(self, now):
        if self.state == PayloadMonitorState.WAITING_FOR_COMMAND_UPLOAD:
            if (
                self.monitor_start_time is not None
                and now - self.monitor_start_time > self.config.execution_timeout_s
            ):
                return self._timeout('command upload confirmation timeout')

        if self.state == PayloadMonitorState.WAITING_FOR_COMMAND_REACHED:
            if (
                self.execution_window_armed
                and self.execution_window_started_at is not None
                and now - self.execution_window_started_at
                > self.config.execution_timeout_s
            ):
                return self._timeout(
                    f'command execution confirmation timeout seq={self.release_command_seq}'
                )

        if self.state == PayloadMonitorState.WAITING_FOR_PWM_CONFIRMATION:
            if now - self.execution_start_time > self.config.execution_timeout_s:
                return self._timeout(
                    f'PWM confirmation timeout channel={self.config.servo_channel} '
                    f'expected={self.config.release_pwm}'
                )
            if (
                self.last_rc_out_time is None
                or now - self.last_rc_out_time > self.config.telemetry_stale_timeout_s
            ):
                age = (
                    float('inf')
                    if self.last_rc_out_time is None
                    else now - self.last_rc_out_time
                )
                return self._throttled(
                    'rc_telemetry_stale', now, 'warning',
                    f'RC output telemetry stale age={age:.2f}s '
                    f'limit={self.config.telemetry_stale_timeout_s:.2f}s'
                )
        return []

    def _timeout(self, message):
        self.state = PayloadMonitorState.TIMEOUT
        return self._once('timeout:' + message, 'error', message)

    def _once(self, key, level, message):
        if key in self._emitted:
            return []
        self._emitted.add(key)
        return [MonitorEvent(level, self.state, message, key)]

    def _throttled(self, key, now, level, message):
        last = self._warning_times.get(key)
        if last is not None and now - last < self.config.warning_throttle_s:
            return []
        self._warning_times[key] = now
        return [MonitorEvent(level, self.state, message, key)]
