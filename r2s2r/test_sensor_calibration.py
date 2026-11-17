import unittest
from pathlib import Path
import numpy as np
import yaml
from r2s2r.core.observation import rigid_transform

ROOT = Path(__file__).resolve().parents[1]

class SensorCalibrationTest(unittest.TestCase):
    def test_selected_profiles_match_bridge_extrinsics(self):
        config = yaml.safe_load((ROOT/"r2s2r/configs/config.yaml").read_text())
        suite = config["sensor_suite"]
        profiles = {"kinematic_car": suite["calibration_dir"],
                    "go2": suite["calibration_dir_by_robot"]["go2"]}
        for robot, relative in profiles.items():
            with self.subTest(robot=robot):
                directory = ROOT/"r2s2r/configs"/relative
                ext = yaml.safe_load((directory/"extrinsics.yaml").read_text())
                profile = "odin1_go2" if robot == "go2" else "odin1"
                bridge = yaml.safe_load((ROOT/f"ros1_ws/src/sim_bridge/config/tf_{profile}.yaml").read_text())
                def transform(t):
                    return rigid_transform(t["translation"],t["rotation"])
                def bridge_transform(prefix):
                    xyzw = bridge[prefix+"_rotation_xyzw"]
                    return rigid_transform(bridge[prefix+"_offset_xyz"],[xyzw[3],*xyzw[:3]])
                mount = transform(ext["ref_imu"]["mount_extrinsics"])
                np.testing.assert_allclose(mount,bridge_transform("sensor_base"),atol=1e-10)
                np.testing.assert_allclose(mount,bridge_transform("imu"),atol=1e-10)
                for name,prefix in [("lidar1","lidar"),("camera1","camera")]:
                    actual = mount @ transform(ext["sensors"][name]["extrinsics"])
                    expected = bridge_transform(prefix)
                    if prefix == "lidar":
                        expected = mount @ expected
                        desired = [.285,0,.12] if robot == "go2" else [.43,0,.10]
                        np.testing.assert_allclose(actual[:3,3],desired,atol=1e-10)
                    np.testing.assert_allclose(actual,expected,atol=1e-10)
                    intrinsics = directory/ext["sensors"][name]["intrinsics_file"]
                    self.assertTrue(intrinsics.is_file(),str(intrinsics))

    def test_standalone_go2_selects_go2_calibration(self):
        config = yaml.safe_load((ROOT/"r2s2r/configs/go2.yaml").read_text())
        self.assertEqual(config["sensor_suite"]["calibration_dir"],"calibration/odin1_go2")

if __name__ == "__main__":
    unittest.main()
