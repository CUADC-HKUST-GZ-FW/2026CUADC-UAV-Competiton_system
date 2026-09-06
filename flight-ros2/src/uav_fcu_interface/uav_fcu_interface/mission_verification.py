"""验证飞控回读任务是否与程序准备上传的任务一致。

本文件不负责构建、上传或执行任务，也不会发送任何飞控控制命令。
它只比较两个航点列表：expected 是程序准备上传的任务，actual 是通过
MAVROS 从飞控 pull-back 得到的任务。这样可以避免只根据
WaypointPush.success 和 transferred 数量误判上传成功。
"""

import math


MAV_CMD_DO_SET_SERVO = 183


def _compare_exact(seq, field, expected, actual):
    """比较命令、坐标系和布尔标志等必须完全一致的字段。"""
    if expected != actual:
        return False, (
            f'seq={seq} field={field} expected={expected} actual={actual}'
        )
    return True, ''


def _compare_float(seq, field, expected, actual, tolerance):
    """在指定容差内比较坐标、高度、PWM和航点参数等浮点字段。"""
    expected = float(expected)
    actual = float(actual)
    if not math.isfinite(expected) or not math.isfinite(actual):
        return False, (
            f'seq={seq} field={field} expected={expected} actual={actual}'
        )
    if not math.isclose(expected, actual, rel_tol=0.0, abs_tol=tolerance):
        return False, (
            f'seq={seq} field={field} expected={expected} actual={actual} '
            f'tolerance={tolerance}'
        )
    return True, ''


def verify_mission_waypoints(expected, actual):
    """逐项比较预期任务和飞控回读任务，返回 ``(成功, 原因)``。"""
    # 数量不同意味着飞控丢失、增加或拒绝了某个任务项。
    if len(expected) != len(actual):
        return False, f'count expected={len(expected)} actual={len(actual)}'
    if not expected:
        return False, 'HOME item missing: mission is empty'

    # FCU seq 0 具有 HOME 语义，因此单独验证，不把它当作普通航点A。
    for field in ('command', 'frame'):
        ok, reason = _compare_exact(
            0, field, getattr(expected[0], field), getattr(actual[0], field)
        )
        if not ok:
            return ok, reason
    # ArduPilot may recalculate HOME altitude when a mission is stored.  HOME
    # z_alt is therefore not stable pull-back content, while its command,
    # frame and geographic position remain verified.  Non-HOME waypoint
    # altitude is still checked below.
    for field, tolerance in {
        'x_lat': 1.0e-7,
        'y_long': 1.0e-7,
    }.items():
        ok, reason = _compare_float(
            0,
            field,
            getattr(expected[0], field),
            getattr(actual[0], field),
            tolerance,
        )
        if not ok:
            return ok, reason

    # seq 1 以后是实际任务项：A、B、R、可选舵机命令、C、D。
    for seq, (expected_wp, actual_wp) in enumerate(
        zip(expected[1:], actual[1:]), start=1
    ):
        # is_current 是飞控当前执行状态，不是稳定的任务内容。执行起点由
        # WaypointSetCurrent 单独设置和确认，因此这里不比较 is_current。
        for field in ('command', 'frame', 'autocontinue'):
            if (
                field == 'frame'
                and int(expected_wp.command) == MAV_CMD_DO_SET_SERVO
                and int(actual_wp.command) == MAV_CMD_DO_SET_SERVO
            ):
                # ArduPilot/MAVROS may normalize a non-positional command's
                # frame during pull-back. Its command and parameters remain
                # strictly verified below.
                continue
            ok, reason = _compare_exact(
                seq,
                field,
                getattr(expected_wp, field),
                getattr(actual_wp, field),
            )
            if not ok:
                return ok, reason

        # 经纬度和高度允许微小的编码/回读误差；DO_SET_SERVO 的PWM使用
        # 0.5us容差，其余任务参数使用更严格的0.05容差。
        float_fields = {
            'param1': 0.05,
            'param2': (
                0.5
                if int(expected_wp.command) == MAV_CMD_DO_SET_SERVO
                else 0.05
            ),
            'param3': 0.05,
            'param4': 0.05,
            'x_lat': 1.0e-7,
            'y_long': 1.0e-7,
            'z_alt': 0.20,
        }
        for field, tolerance in float_fields.items():
            ok, reason = _compare_float(
                seq,
                field,
                getattr(expected_wp, field),
                getattr(actual_wp, field),
                tolerance,
            )
            if not ok:
                return ok, reason

    return True, f'count={len(expected)} content_match=true'
