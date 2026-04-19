import struct
import unittest

np = skip_reason = None
try:
    import numpy as np
except ImportError:
    skip_reason = "numpy is not installed"

try:
    import rpi_tilt_3d as host
except ImportError as exc:
    if skip_reason is None:
        raise
    host = None


@unittest.skipIf(skip_reason is not None, skip_reason)
class ProtocolTests(unittest.TestCase):
    def test_frame_round_trip_for_status(self):
        payload = struct.pack("<BHH", 1, 3, 46)
        frame = host.encode_frame(host.MSG_SCAN_STATUS, payload)

        self.assertEqual(frame[0], host.SYNC1)
        self.assertEqual(frame[1], host.SYNC2)
        self.assertEqual(frame[2], host.MSG_SCAN_STATUS)
        self.assertEqual(frame[-1], host.frame_checksum(host.MSG_SCAN_STATUS, payload))

        decoded = host.decode_payload(host.MSG_SCAN_STATUS, payload)
        self.assertEqual(decoded.state, 1)
        self.assertEqual(decoded.step, 3)
        self.assertEqual(decoded.total, 46)

    def test_scan_slice_decode(self):
        payload = struct.pack("<fffH", 10.0, 1.0, -2.0, 2)
        payload += struct.pack("<HHB", 9000, 1000, 20)
        payload += struct.pack("<HHB", 18000, 2000, 30)

        decoded = host.decode_payload(host.MSG_SCAN_SLICE, payload)

        self.assertAlmostEqual(decoded.tilt_deg, 10.0)
        np.testing.assert_allclose(decoded.angles_deg, np.array([90.0, 180.0]))
        np.testing.assert_array_equal(decoded.dists_mm, np.array([1000, 2000], dtype=np.uint16))
        np.testing.assert_array_equal(decoded.qualities, np.array([20, 30], dtype=np.uint8))

    def test_transform_filters_and_projects(self):
        angles = np.array([0.0, 90.0, 180.0])
        dists = np.array([1000, 0, 2000], dtype=np.uint16)
        qualities = np.array([10, 10, 0], dtype=np.uint8)

        pts = host.transform_slice_to_3d(
            angles,
            dists,
            tilt_deg=0.0,
            qualities=qualities,
            min_quality=1,
        )

        self.assertEqual(pts.shape, (1, 3))
        np.testing.assert_allclose(pts[0], np.array([1.0, 0.0, 0.0]), atol=1e-9)

    def test_tilt_projection(self):
        pts = host.transform_slice_to_3d(
            np.array([0.0]),
            np.array([1000], dtype=np.uint16),
            tilt_deg=90.0,
            max_dist_mm=2000,
        )

        np.testing.assert_allclose(pts[0], np.array([0.0, 0.0, -1.0]), atol=1e-9)


if __name__ == "__main__":
    unittest.main()
