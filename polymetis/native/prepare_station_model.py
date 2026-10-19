"""Export the controller-reported arm, configured payload and TCP for offline review.

This reads saved inspection files only. A configured payload is not evidence that
the physical camera, bracket, fingers or held object have been calibrated.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np


def vector(value, size):
    result = np.asarray(value, dtype=np.float64)
    if result.shape != (size,) or not np.all(np.isfinite(result)):
        raise ValueError("Invalid inspection vector")
    return result


def numbers(value):
    return " ".join(format(float(item), ".17g") for item in value)


def prepare(inspection, output):
    snapshot = json.loads((inspection / "inspection.json").read_text())
    state = snapshot["robot_state"]
    reported = vector(state["O_T_EE"], 16).reshape((4, 4), order="F")
    calculated = vector(snapshot["O_T_ee_model"], 16).reshape((4, 4), order="F")
    if np.linalg.norm(reported[:3, 3] - calculated[:3, 3]) > .001 or np.max(np.abs(reported[:3, :3] - calculated[:3, :3])) > .001:
        raise ValueError("Inspection model does not agree with controller pose; repair libfranka first")
    tcp = vector(state["F_T_EE"], 16).reshape((4, 4), order="F")
    rotation = tcp[:3, :3]
    if not np.allclose(tcp[3], [0, 0, 0, 1], atol=1e-7) or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6) or abs(np.linalg.det(rotation) - 1) > 1e-6:
        raise ValueError("Invalid flange-to-TCP transform")
    # Project float-precision controller rotation onto SO(3) before URDF export.
    u, _, v = np.linalg.svd(rotation)
    rotation = u @ v
    pitch = math.atan2(-rotation[2, 0], math.hypot(rotation[0, 0], rotation[1, 0]))
    if abs(math.cos(pitch)) < 1e-8:
        roll = 0.0
        yaw = math.atan2(-rotation[0, 1], rotation[1, 1])
    else:
        roll = math.atan2(rotation[2, 1], rotation[2, 2])
        yaw = math.atan2(rotation[1, 0], rotation[0, 0])
    mass = float(state["m_total"])
    com = vector(state["F_x_Ctotal"], 3)
    inertia = vector(state["I_total"], 9).reshape((3, 3), order="F")
    eigenvalues = np.linalg.eigvalsh(inertia)
    if not math.isfinite(mass) or mass < 0 or not np.allclose(inertia, inertia.T, atol=1e-10) or eigenvalues[0] < -1e-10 or eigenvalues[-1] > eigenvalues[:2].sum() + 1e-8:
        raise ValueError("Invalid configured payload inertia")
    if mass == 0 and (np.any(inertia) or np.any(com)):
        raise ValueError("Massless payload has nonzero inertia or center of mass")
    raw = (inspection / "robot-model.raw").read_bytes()
    root = ET.fromstring(raw)
    joints = [joint for joint in root.findall("joint") if joint.get("type") != "fixed"]
    if [joint.get("name") for joint in joints] != ["joint" + str(index) for index in range(1, 8)]:
        raise ValueError("Expected the robot-provided seven-joint Franka arm")
    flange = root.find("link[@name='link8']")
    if flange is None or flange.find("inertial") is not None:
        raise ValueError("Expected bare flange link8 without an existing payload")
    if root.find("link[@name='franka_tool']") is not None:
        raise ValueError("TCP link already present")
    for link in root.findall("link"):
        for child in list(link):
            if child.tag in ("visual", "collision"):
                link.remove(child)
    payload = ET.SubElement(root, "link", name="franka_configured_payload")
    if mass > 0:
        inertial = ET.SubElement(payload, "inertial")
        ET.SubElement(inertial, "origin", xyz=numbers(com), rpy="0 0 0")
        ET.SubElement(inertial, "mass", value=str(mass))
        ET.SubElement(inertial, "inertia", **{name: str(inertia[i, j]) for name, i, j in
            (("ixx",0,0), ("iyy",1,1), ("izz",2,2), ("ixy",0,1), ("ixz",0,2), ("iyz",1,2))})
    joint = ET.SubElement(root, "joint", name="configured_payload_joint", type="fixed")
    ET.SubElement(joint, "parent", link="link8")
    ET.SubElement(joint, "child", link="franka_configured_payload")
    ET.SubElement(joint, "origin", xyz="0 0 0", rpy="0 0 0")
    ET.SubElement(root, "link", name="franka_tool")
    joint = ET.SubElement(root, "joint", name="configured_tcp_joint", type="fixed")
    ET.SubElement(joint, "parent", link="link8")
    ET.SubElement(joint, "child", link="franka_tool")
    ET.SubElement(joint, "origin", xyz=numbers(tcp[:3, 3]), rpy=numbers([roll, pitch, yaw]))
    model = ET.tostring(root, encoding="utf-8", xml_declaration=True)
    provenance = {
        "robot_name": root.get("name"), "ee_link_name": "franka_tool",
        "source_urdf_sha256": hashlib.sha256(raw).hexdigest(),
        "inspection_sha256": hashlib.sha256((inspection / "inspection.json").read_bytes()).hexdigest(),
        "urdf_sha256": hashlib.sha256(model).hexdigest(),
        "configured_mass_kg": mass, "payload_physically_verified": False, "model_verified": False,
        "joint_names": [joint.get("name") for joint in joints],
        "joint_lower": [float(joint.find("limit").get("lower")) for joint in joints],
        "joint_upper": [float(joint.find("limit").get("upper")) for joint in joints],
        "notes": "Controller-configured payload only; no collision geometry; offline cross-validation and physical payload verification required.",
    }
    output.mkdir(parents=True, exist_ok=False)
    (output / "station_arm.urdf").write_bytes(model)
    (output / "provenance.json").write_text(json.dumps(provenance, indent=2) + "\n")
    return provenance


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("inspection", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    print(json.dumps(prepare(args.inspection, args.output), indent=2))
