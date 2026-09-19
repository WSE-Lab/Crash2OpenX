"""Coordinate-sign regression and measurement-noise preservation."""
import importlib.util
from pathlib import Path
import unittest

import numpy as np

root = Path(__file__).parents[1]
source = root / "runtime_overrides/gnss_compat.py"
if not source.is_file():
    source = root / "scenario_runner/srunner/scenariomanager/actorcontrols/gnss_compat.py"
spec = importlib.util.spec_from_file_location("gnss_compat", source)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class GnssCompatibilityTest(unittest.TestCase):
    def test_both_native_y_conventions_and_noise_are_preserved(self):
        for sign in (-1, 1):
            with self.subTest(sign=sign):
                sample = lambda x, y: [sign * y / 111319.49, x / 111319.49]
                calibration = module.MapGnssCalibration(sample, (0, 500, -200, 200))
                self.assertLess(calibration.max_probe_error_m, 1e-8)
                np.testing.assert_allclose(calibration.world_xy(sample(100.25, 1.4)), [100.25, 1.4], atol=1e-8)
                converted = calibration.route_gps(sample(100.25, 1.4))
                self.assertLess(converted[0], 0)
                self.assertGreater(converted[1], 0)
                # A 0.4 m perturbation in the sensor survives conversion.
                noisy = calibration.route_gps(sample(100.25, 1.8))
                self.assertAlmostEqual((noisy[0] - converted[0]) * 111319.49, -.4, places=6)

    def test_rejects_inaccurate_local_calibration(self):
        with self.assertRaisesRegex(ValueError, "nonlinear"):
            module.MapGnssCalibration(lambda x, y: [y / 1e5, (x + x * x / 1e3) / 1e5], (0, 500, -200, 200))


if __name__ == "__main__":
    unittest.main()
