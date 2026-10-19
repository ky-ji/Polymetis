import copy
import json
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET

from prepare_station_model import prepare


class StationModelTest(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name)
        self.source = self.root / "inspection"
        self.source.mkdir()
        identity = [1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1]
        self.snapshot = {"O_T_ee_model": identity.copy(), "robot_state": {
            "O_T_EE": identity.copy(), "F_T_EE": identity.copy(), "m_total": .73,
            "F_x_Ctotal": [-.01,0,.03], "I_total": [.001,0,0,0,.0025,0,0,0,.0017]}}
        arm = ET.Element("robot", name="test_arm")
        for index in range(9):
            link = ET.SubElement(arm, "link", name="link" + str(index))
            ET.SubElement(link, "visual")
        for index in range(1,8):
            joint = ET.SubElement(arm, "joint", name="joint"+str(index), type="revolute")
            ET.SubElement(joint, "parent", link="link"+str(index-1))
            ET.SubElement(joint, "child", link="link"+str(index))
            ET.SubElement(joint, "limit", lower="-2", upper="2")
        (self.source / "robot-model.raw").write_bytes(ET.tostring(arm))

    def run_prepare(self, snapshot=None):
        (self.source / "inspection.json").write_text(json.dumps(snapshot or self.snapshot))
        return prepare(self.source, self.root / "output")

    def test_preserves_payload_and_marks_unverified(self):
        self.snapshot["robot_state"]["F_T_EE"] = self.snapshot["robot_state"]["F_T_EE"].copy()
        self.snapshot["robot_state"]["F_T_EE"][14] = .1
        provenance = self.run_prepare()
        model = ET.parse(self.root / "output/station_arm.urdf").getroot()
        self.assertFalse(provenance["model_verified"])
        self.assertFalse(provenance["payload_physically_verified"])
        self.assertEqual(provenance["configured_mass_kg"], .73)
        self.assertEqual(model.find("joint[@name='configured_tcp_joint']/origin").get("xyz"), "0 0 0.10000000000000001")
        self.assertIsNone(model.find(".//visual"))
        self.assertEqual(float(model.find("link[@name='franka_configured_payload']/inertial/mass").get("value")), .73)

    def test_bad_pose_is_rejected_before_writing(self):
        snapshot = copy.deepcopy(self.snapshot)
        snapshot["O_T_ee_model"][12] = .01
        with self.assertRaises(ValueError): self.run_prepare(snapshot)
        self.assertFalse((self.root / "output").exists())

    def test_invalid_payloads(self):
        for key, value in [("m_total", float("nan")), ("m_total", -1),
                           ("F_x_Ctotal", [float("inf"),0,0]),
                           ("I_total", [-1,0,0,0,1,0,0,0,1]),
                           ("I_total", [1,0,0,0,1,0,0,0,4]),
                           ("m_total", 0)]:
            with self.subTest(key=key, value=value):
                snapshot = copy.deepcopy(self.snapshot)
                snapshot["robot_state"][key] = value
                with self.assertRaises(ValueError): self.run_prepare(snapshot)

    def test_reflection_is_not_a_tcp_rotation(self):
        snapshot = copy.deepcopy(self.snapshot)
        snapshot["robot_state"]["F_T_EE"][0] = -1
        with self.assertRaises(ValueError): self.run_prepare(snapshot)

    def test_existing_output_not_overwritten(self):
        self.run_prepare()
        with self.assertRaises(FileExistsError): self.run_prepare()


if __name__ == "__main__":
    unittest.main()
