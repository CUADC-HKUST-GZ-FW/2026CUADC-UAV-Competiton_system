"""Draw aircraft lat/lon/altitude onto a BGR frame."""

import math

import cv2


def _text(frame, text, origin, color=(255, 255, 255), scale=0.55, thickness=1):
    cv2.putText(
        frame,
        text,
        origin,
        cv2.FONT_HERSHEY_SIMPLEX,
        scale,
        (0, 0, 0),
        thickness + 2,
        cv2.LINE_AA,
    )
    cv2.putText(
        frame,
        text,
        origin,
        cv2.FONT_HERSHEY_SIMPLEX,
        scale,
        color,
        thickness,
        cv2.LINE_AA,
    )


def _fmt(value, digits=1, suffix=''):
    if value is None or (isinstance(value, float) and not math.isfinite(value)):
        return 'na'
    return f'{value:.{digits}f}{suffix}'


def draw_geo_overlay(frame, telemetry, targets, match_age_sec=None):
    """In-place HUD with aircraft coordinates. ``targets`` is optional detection text."""
    if frame is None or frame.size == 0:
        return frame

    panel_h = 76 + 22 * max(1, len(targets or []))
    cv2.rectangle(frame, (0, 0), (min(frame.shape[1], 980), panel_h), (0, 0, 0), cv2.FILLED)

    lat = telemetry.latitude if telemetry else None
    lon = telemetry.longitude if telemetry else None
    rel_alt = telemetry.relative_alt_m if telemetry else None
    amsl = telemetry.amsl_m if telemetry else None
    source = telemetry.source if telemetry else 'none'
    age_ms = None if match_age_sec is None else match_age_sec * 1000.0

    _text(
        frame,
        f'LAT {_fmt(lat, 7)}  LON {_fmt(lon, 7)}',
        (8, 24),
        (80, 255, 80),
        0.6,
        2,
    )
    _text(
        frame,
        (
            f'REL_ALT {_fmt(rel_alt, 2, "m")}  '
            f'AMSL {_fmt(amsl, 2, "m")}  '
            f'age {_fmt(age_ms, 0, "ms")}  '
            f'src {source}'
        ),
        (8, 48),
        (220, 220, 220),
        0.5,
        1,
    )

    if not targets:
        _text(frame, 'no target this frame', (8, 72), (180, 180, 180))
        return frame

    for index, target in enumerate(targets):
        label = target.get('label', '?')
        prob = target.get('prob')
        y = 72 + index * 22
        _text(frame, f'{label} p={_fmt(prob, 2)}', (8, y), (0, 220, 255))
        pixel = target.get('pixel')
        if pixel is not None:
            point = (int(round(pixel[0])), int(round(pixel[1])))
            cv2.circle(frame, point, 7, (0, 255, 255), 2, cv2.LINE_AA)
    return frame
