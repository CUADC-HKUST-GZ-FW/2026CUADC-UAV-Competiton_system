import unittest

from uav_payload.servo_open_detector import (
    ServoOpenDetector,
    ServoOpenDetectorConfig,
    ServoOpenState,
)


class ServoOpenDetectorTest(unittest.TestCase):
    def make_detector(self, **overrides):
        values = {
            'release_pwm': 1900,
            'safe_pwm': 1350,
            'pwm_tolerance_us': 20,
            'required_open_samples': 3,
            'required_closed_samples': 3,
        }
        values.update(overrides)
        return ServoOpenDetector(ServoOpenDetectorConfig(**values))

    def arm(self, detector):
        events = []
        for now in (0.0, 0.1, 0.2):
            events.extend(detector.observe(1350, now))
        self.assertEqual(ServoOpenState.ARMED, detector.state)
        self.assertEqual(['armed'], [event.kind for event in events])

    def test_requires_closed_state_before_first_open(self):
        detector = self.make_detector()

        events = []
        for now in (0.0, 0.1, 0.2):
            events.extend(detector.observe(1900, now))

        self.assertEqual([], events)
        self.assertEqual(ServoOpenState.WAITING_FOR_CLOSED, detector.state)

    def test_three_release_samples_confirm_opening(self):
        detector = self.make_detector()
        self.arm(detector)

        events = []
        for now, pwm in ((1.0, 1890), (1.1, 1905), (1.2, 1900)):
            events.extend(detector.observe(pwm, now))

        self.assertEqual(
            ['candidate_started', 'open_confirmed'],
            [event.kind for event in events],
        )
        confirmed = events[-1]
        self.assertEqual(3, confirmed.sample_count)
        self.assertAlmostEqual(0.2, confirmed.duration_s)
        self.assertEqual(ServoOpenState.WAITING_FOR_CLOSE, detector.state)

    def test_transient_candidate_is_rejected(self):
        detector = self.make_detector()
        self.arm(detector)

        detector.observe(1900, 1.0)
        events = detector.observe(1700, 1.1)

        self.assertEqual(['candidate_rejected'], [event.kind for event in events])
        self.assertEqual(ServoOpenState.ARMED, detector.state)

    def test_confirmed_close_rearms_for_another_open(self):
        detector = self.make_detector()
        self.arm(detector)
        for now in (1.0, 1.1, 1.2):
            detector.observe(1900, now)

        events = []
        for now in (2.0, 2.1, 2.2):
            events.extend(detector.observe(1350, now))

        self.assertEqual(['rearmed'], [event.kind for event in events])
        self.assertEqual(ServoOpenState.ARMED, detector.state)
        second = detector.observe(1900, 3.0)
        self.assertEqual(['candidate_started'], [event.kind for event in second])

    def test_jitter_inside_tolerance_still_confirms(self):
        detector = self.make_detector()
        self.arm(detector)

        events = []
        for now, pwm in ((1.0, 1880), (1.1, 1920), (1.2, 1895)):
            events.extend(detector.observe(pwm, now))

        self.assertEqual('open_confirmed', events[-1].kind)


if __name__ == '__main__':
    unittest.main()
