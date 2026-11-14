from __future__ import division
import unittest
import os
import sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "src"))
from shoot_race.imu_ready import FreshStream


class ReadinessTests(unittest.TestCase):
    def fill(self, stream, start=100.0):
        for i in range(12):
            t = start + i * 0.1
            stream.observe(t, t, t)

    def test_no_data(self):
        self.assertFalse(FreshStream().ready(100, 100))

    def test_stable_then_stale(self):
        s = FreshStream()
        self.fill(s)
        self.assertTrue(s.ready(101.1, 101.1))
        self.assertFalse(s.ready(102, 102))

    def test_single_or_duplicate_message_not_ready(self):
        s = FreshStream()
        for i in range(20):
            s.observe(100+i*.01, 100, 100+i*.01)
        self.assertFalse(s.ready(100.2, 100.2))

    def test_old_and_future_stamps_rejected(self):
        for offset in [-3, 3]:
            s = FreshStream()
            for i in range(20):
                s.observe(100+i*.1, 100+i*.1+offset, 100+i*.1)
            self.assertFalse(s.ready(102, 102))

    def test_interruption_requires_new_stable_window(self):
        s = FreshStream()
        self.fill(s)
        s.observe(103, 103, 103)
        self.assertFalse(s.ready(103, 103))
        self.fill(s, 103)
        self.assertTrue(s.ready(104.1, 104.1))

    def test_invalid_data_resets(self):
        s = FreshStream()
        self.fill(s)
        s.observe(101.2, 101.2, 101.2, valid=False)
        self.assertFalse(s.ready(101.2, 101.2))

    def test_backward_stamp_restarts_window(self):
        s = FreshStream()
        self.fill(s)
        s.observe(101.2, 101.0, 101.2)
        self.assertFalse(s.ready(101.2, 101.2))


if __name__ == '__main__':
    unittest.main()
