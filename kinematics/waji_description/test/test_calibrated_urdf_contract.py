"""Regression contract for the integrated measured excavator URDF."""

from pathlib import Path
import unittest
import xml.etree.ElementTree as ET


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
URDF_PATH = PACKAGE_ROOT / "urdf" / "waji.urdf"


def _joint(root: ET.Element, name: str) -> ET.Element:
    joint = root.find(f"./joint[@name='{name}']")
    if joint is None:
        raise AssertionError(f"missing URDF joint: {name}")
    return joint


def _xyz_or_rpy(joint: ET.Element, attribute: str) -> tuple[float, float, float]:
    origin = joint.find("origin")
    if origin is None:
        raise AssertionError(f"joint {joint.attrib['name']} has no origin")
    return tuple(float(value) for value in origin.attrib[attribute].split())


class CalibratedUrdfContractTest(unittest.TestCase):
    def setUp(self):
        self.root = ET.parse(URDF_PATH).getroot()

    def test_fk_chain_uses_current_machine_geometry_and_encoder_zeroes(self):
        # Change these integrated measured values only after a documented survey.
        expected = {
            "fk_root_to_base": ((-0.06, 0.0, -0.18), (0.0, 0.0, 0.0)),
            "swing_joint": ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
            "boom_joint": ((0.05, 0.0, 0.0), (0.0, 0.52, 0.0)),
            "arm_joint": ((0.78, 0.0, 0.0), (0.0, 2.95, 0.0)),
            "bucket_joint": ((0.35, 0.0, 0.0), (0.0, 3.67, 0.0)),
            "bucket_to_tip": ((0.2, 0.0, 0.0), (0.0, 1.1, 0.0)),
        }

        for name, (expected_xyz, expected_rpy) in expected.items():
            joint = _joint(self.root, name)
            self.assertEqual(_xyz_or_rpy(joint, "xyz"), expected_xyz, name)
            self.assertEqual(_xyz_or_rpy(joint, "rpy"), expected_rpy, name)

    def test_joint_axes_encode_the_existing_negative_sensor_signs(self):
        expected_axes = {
            "swing_joint": (0.0, 0.0, -1.0),
            "boom_joint": (0.0, -1.0, 0.0),
            "arm_joint": (0.0, -1.0, 0.0),
            "bucket_joint": (0.0, -1.0, 0.0),
        }
        for name, expected_axis in expected_axes.items():
            axis = _joint(self.root, name).find("axis")
            self.assertIsNotNone(axis, name)
            actual_axis = tuple(float(value) for value in axis.attrib["xyz"].split())
            self.assertEqual(actual_axis, expected_axis, name)


if __name__ == "__main__":
    unittest.main()
