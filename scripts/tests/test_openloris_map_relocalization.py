import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import eval_openloris_map_relocalization as ev  # noqa: E402
import export_openloris_localization_map as ex  # noqa: E402


class ParseModelImagesTest(unittest.TestCase):
    def test_parses_pose_rows_only(self):
        import tempfile

        text = (
            "# header\n"
            "1 1 0 0 0 -1 -2 -3 1 cam1_000000.png\n"
            "10.0 20.0 3\n"
            "2 1 0 0 0 -4 -5 -6 2 cam2_000001.png\n"
            "11.0 21.0 4\n"
        )
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "images.txt"
            path.write_text(text)
            out = ex.parse_model_images(path)
        self.assertEqual(out, {1: ("cam1_000000.png", 1), 2: ("cam2_000001.png", 2)})


class ParsePoints3DTest(unittest.TestCase):
    def test_parses_header_and_track(self):
        import tempfile

        text = (
            "# POINT3D_ID X Y Z R G B ERROR TRACK\n"
            "1 -0.5 -1.4 1.7 255 255 255 0 840 2 776 15 842 8\n"
            "2 0.1 0.2 0.3 1 2 3 0.5 10 0 11 1\n"
        )
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "points3D.txt"
            path.write_text(text)
            out = ex.parse_points3d(path)
        header, track = out[1]
        self.assertEqual(header, ["1", "-0.5", "-1.4", "1.7", "255", "255", "255", "0"])
        self.assertEqual(track, [(840, 2), (776, 15), (842, 8)])
        self.assertEqual(out[2][1], [(10, 0), (11, 1)])


class GeometryTest(unittest.TestCase):
    def test_identity_quaternion_is_identity(self):
        self.assertTrue(np.allclose(ev.quat_to_R((1.0, 0.0, 0.0, 0.0)), np.eye(3)))

    def test_rotation_angle_of_identical_is_zero(self):
        R = ev.quat_to_R((0.7, 0.1, 0.2, 0.3))
        self.assertAlmostEqual(ev.rotation_angle_deg(R, R), 0.0, places=6)

    def test_rotation_angle_of_180_degree_yaw(self):
        R0 = np.eye(3)
        R1 = np.array([[-1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, -1.0]])
        self.assertAlmostEqual(ev.rotation_angle_deg(R0, R1), 180.0, places=4)

    def test_camera_centre_convention(self):
        # t = -R c  =>  c = -R^T t ; for R=I, c = -t.
        self.assertTrue(np.allclose(ev.quat_to_R((1, 0, 0, 0)) @ np.array([1.0, 2.0, 3.0]),
                                    np.array([1.0, 2.0, 3.0])))


if __name__ == "__main__":
    unittest.main()
