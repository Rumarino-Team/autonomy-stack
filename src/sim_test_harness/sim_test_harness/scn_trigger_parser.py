"""Parse <volumetric_trigger> elements from Stonefish .scn XML files."""

import xml.etree.ElementTree as ET
from dataclasses import dataclass


@dataclass
class VolumeTrigger:
    name: str
    position: tuple  # (x, y, z)
    rotation_rpy: tuple  # (roll, pitch, yaw)
    dimensions: tuple  # (x, y, z)
    timeout_s: float


def _parse_vector3(text: str) -> tuple:
    parts = text.strip().split()
    if len(parts) != 3:
        raise ValueError(f"Expected 3 space-separated floats, got: '{text}'")
    return (float(parts[0]), float(parts[1]), float(parts[2]))


def parse_triggers(scn_file_path: str) -> list:
    """Parse all <volumetric_trigger> elements from a .scn file.

    Args:
        scn_file_path: Absolute path to the .scn XML file.

    Returns:
        List of VolumeTrigger instances found in the file.
    """
    tree = ET.parse(scn_file_path)
    root = tree.getroot()

    triggers = []
    for elem in root.findall("volumetric_trigger"):
        name = elem.get("name", f"trigger_{len(triggers)}")
        timeout_s = float(elem.get("timeout_s", "30.0"))

        origin = elem.find("origin")
        if origin is not None:
            position = _parse_vector3(origin.get("xyz", "0 0 0"))
            rotation_rpy = _parse_vector3(origin.get("rpy", "0 0 0"))
        else:
            position = (0.0, 0.0, 0.0)
            rotation_rpy = (0.0, 0.0, 0.0)

        dims = elem.find("dimensions")
        if dims is not None:
            dimensions = _parse_vector3(dims.get("xyz", "1 1 1"))
        else:
            dimensions = (1.0, 1.0, 1.0)

        triggers.append(VolumeTrigger(
            name=name,
            position=position,
            rotation_rpy=rotation_rpy,
            dimensions=dimensions,
            timeout_s=timeout_s,
        ))

    return triggers
