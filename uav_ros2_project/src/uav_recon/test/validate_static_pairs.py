"""Validate the proposed camera extrinsics against the four static pairs."""

import math


PAIRS = (
    ('south-1', 'S', 22.89161591, 113.47704246, 22.8915702, 113.4770577),
    ('south-2', 'S', 22.89158984, 113.47709287, 22.8915479, 113.4771073),
    ('north-1', 'N', 22.89176544, 113.47714195, 22.8916876, 113.4771395),
    ('north-2', 'N', 22.89173397, 113.47716562, 22.8916616, 113.4771623),
)

REFERENCE_LATITUDE_DEG = 22.8916
FIXED_HEIGHT_M = 11.5
CAMERA_UP_OFFSET_M = -0.09
OLD_FORWARD_TILT_DEG = 20.0
NEW_FORWARD_TILT_DEG = 12.0
NEW_LEFT_TILT_DEG = 4.5


def metres_per_degree(latitude_deg):
    latitude = math.radians(latitude_deg)
    north = (
        111132.92
        - 559.82 * math.cos(2.0 * latitude)
        + 1.175 * math.cos(4.0 * latitude)
    )
    east = 111412.84 * math.cos(latitude) - 93.5 * math.cos(3.0 * latitude)
    return east, north


def main():
    east_per_degree, north_per_degree = metres_per_degree(REFERENCE_LATITUDE_DEG)
    errors = []
    for name, heading, solved_lat, solved_lon, truth_lat, truth_lon in PAIRS:
        east = (solved_lon - truth_lon) * east_per_degree
        north = (solved_lat - truth_lat) * north_per_degree
        errors.append((name, heading, east, north))

    common_east = sum(item[2] for item in errors) / len(errors)
    common_north = sum(item[3] for item in errors) / len(errors)

    camera_height_m = FIXED_HEIGHT_M + CAMERA_UP_OFFSET_M
    forward_delta = camera_height_m * (
        math.tan(math.radians(NEW_FORWARD_TILT_DEG))
        / math.cos(math.radians(NEW_LEFT_TILT_DEG))
        - math.tan(math.radians(OLD_FORWARD_TILT_DEG))
    )
    right_delta = -camera_height_m * math.tan(math.radians(NEW_LEFT_TILT_DEG))

    before = []
    after = []
    print(f'common_map_error_m: east={common_east:+.3f}, north={common_north:+.3f}')
    print(f'extrinsic_correction_m: forward={forward_delta:+.3f}, right={right_delta:+.3f}')
    print('pair, old_forward, old_right, new_forward, new_right, new_body_norm')
    for name, heading, east, north in errors:
        residual_east = east - common_east
        residual_north = north - common_north
        if heading == 'N':
            forward, right = residual_north, residual_east
        else:
            forward, right = -residual_north, -residual_east
        new_forward = forward + forward_delta
        new_right = right + right_delta
        before.append(math.hypot(forward, right))
        after.append(math.hypot(new_forward, new_right))
        print(
            f'{name}, {forward:+.3f}, {right:+.3f}, '
            f'{new_forward:+.3f}, {new_right:+.3f}, {after[-1]:.3f}'
        )

    old_rms = math.sqrt(sum(value * value for value in before) / len(before))
    new_rms = math.sqrt(sum(value * value for value in after) / len(after))
    print(f'body_error_rms_m: {old_rms:.3f} -> {new_rms:.3f}')
    print(f'body_error_max_m: {max(before):.3f} -> {max(after):.3f}')

    height_35_camera_m = 35.0 + CAMERA_UP_OFFSET_M
    correction_35_forward = height_35_camera_m * (
        math.tan(math.radians(NEW_FORWARD_TILT_DEG))
        / math.cos(math.radians(NEW_LEFT_TILT_DEG))
        - math.tan(math.radians(OLD_FORWARD_TILT_DEG))
    )
    correction_35_right = -height_35_camera_m * math.tan(math.radians(NEW_LEFT_TILT_DEG))
    print(
        f'35m_extrinsic_correction_m: forward={correction_35_forward:+.3f}, '
        f'right={correction_35_right:+.3f}'
    )


if __name__ == '__main__':
    main()
