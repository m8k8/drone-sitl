"""Generate a Gazebo display scene using the B-D model's motor geometry."""

import math
from pathlib import Path
import xml.etree.ElementTree as ET


WORLD_NAME = "bd_sitl"
MODEL_NAME = "prototype_bd"

_SCENE = f"""
<sdf version="1.11">
  <world name="{WORLD_NAME}">
    <physics name="display" type="ignored">
      <max_step_size>0.01</max_step_size>
      <real_time_factor>1</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics" />
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands" />
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster" />
    <scene>
      <ambient>0.52 0.57 0.64 1</ambient>
      <background>0.71 0.79 0.86 1</background>
      <grid>false</grid>
    </scene>
    <light name="sun" type="directional">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 20 0 0 0</pose>
      <diffuse>0.82 0.82 0.80 1</diffuse>
      <specular>0.24 0.24 0.24 1</specular>
      <direction>-0.4 0.3 -0.87</direction>
    </light>
    <gui>
      <plugin filename="MinimalScene" name="3D View">
        <gz-gui>
          <title>3D View</title>
          <property type="bool" key="showTitleBar">false</property>
          <property type="string" key="state">docked</property>
        </gz-gui>
        <engine>ogre2</engine>
        <scene>scene</scene>
        <ambient_light>0.52 0.57 0.64</ambient_light>
        <background_color>0.71 0.79 0.86</background_color>
        <camera_pose>-9 -14 10 0 0.214 0.971</camera_pose>
        <camera_clip><near>0.05</near><far>200</far></camera_clip>
      </plugin>
      <plugin filename="CameraTracking" name="Camera Tracking">
        <gz-gui>
          <property type="bool" key="resizable">false</property>
          <property type="double" key="width">5</property>
          <property type="double" key="height">5</property>
          <property type="string" key="state">floating</property>
          <property type="bool" key="showTitleBar">false</property>
        </gz-gui>
      </plugin>
    </gui>
    <model name="test_field">
      <static>true</static>
      <link name="ground" />
    </model>
    <model name="{MODEL_NAME}">
      <static>true</static>
      <pose>0 0 0.1 0 0 {math.radians(97):.9f}</pose>
      <link name="airframe" />
    </model>
  </world>
</sdf>
"""


def _numbers(values):
    return " ".join(f"{value:.9g}" for value in values)


def _visual(link, name, pose, shape, dimensions, color, *, transparency=0.0):
    visual = ET.SubElement(link, "visual", name=name)
    ET.SubElement(visual, "pose").text = _numbers(pose)
    geometry = ET.SubElement(visual, "geometry")
    solid = ET.SubElement(geometry, shape)
    for tag, value in dimensions.items():
        ET.SubElement(solid, tag).text = value
    material = ET.SubElement(visual, "material")
    ET.SubElement(material, "ambient").text = color
    ET.SubElement(material, "diffuse").text = color
    if transparency:
        ET.SubElement(visual, "transparency").text = str(transparency)
        ET.SubElement(visual, "cast_shadows").text = "false"


def _add_field(link):
    _visual(link, "surface", (5, 5, -0.04, 0, 0, 0), "box", {"size": "120 120 0.08"},
            "0.32 0.38 0.35 1")
    for offset in range(-15, 31, 5):
        _visual(link, f"grid_e_{offset}", (offset, 7.5, 0.004, 0, 0, 0),
                "box", {"size": "0.035 45 0.002"}, "0.40 0.47 0.44 1")
        _visual(link, f"grid_n_{offset}", (7.5, offset, 0.004, 0, 0, 0),
                "box", {"size": "45 0.035 0.002"}, "0.40 0.47 0.44 1")
    _visual(link, "launch_pad", (0, 0, 0.001, 0, 0, 0), "cylinder",
            {"radius": "1.05", "length": "0.006"}, "0.67 0.69 0.64 1")


def _add_airframe(link, model):
    motors = []
    for index in range(1, 5):
        forward, right, down = model[f"motor{index}_position"]
        motors.append((forward, -right, -down))  # ArduPilot FRD to Gazebo FLU.

    center_x, center_y, motor_height = (sum(axis) / 4 for axis in zip(*motors))
    graphite = "0.08 0.10 0.12 1"
    aluminum = "0.28 0.33 0.39 1"
    nose_color = "0.99 0.45 0.10 1"
    _visual(link, "body", (center_x, center_y, motor_height - 0.02, 0, 0, 0),
            "box", {"size": "0.40 0.32 0.10"}, "0.86 0.88 0.84 1")
    _visual(link, "nose", (center_x + 0.18, center_y, motor_height + 0.036, 0, 0, 0),
            "box", {"size": "0.075 0.19 0.022"}, nose_color)

    for index, (x, y, z) in enumerate(motors, 1):
        dx = x - center_x
        dy = y - center_y
        arm_pose = (center_x + dx / 2, center_y + dy / 2, motor_height,
                    0, math.pi / 2, math.atan2(dy, dx))
        _visual(link, f"arm_{index}", arm_pose, "cylinder",
                {"radius": "0.015", "length": _numbers((math.hypot(dx, dy),))}, graphite)
        _visual(link, f"motor_{index}", (x, y, z, 0, 0, 0),
                "cylinder", {"radius": "0.044", "length": "0.075"}, aluminum)
        _visual(link, f"propeller_{index}", (x, y, z + 0.044, 0, 0, math.pi / 4 * (-1) ** index),
                "box", {"size": "0.6096 0.032 0.008"}, graphite)
        _visual(link, f"rotor_disk_{index}", (x, y, z + 0.044, 0, 0, 0),
                "cylinder", {"radius": "0.3048", "length": "0.002"}, aluminum, transparency=0.83)

    # The skids are schematic and meet the native model's 0.10 m ground height.
    for side, y in (("left", 0.145), ("right", -0.145)):
        _visual(link, f"skid_{side}", (center_x, center_y + y, -0.091, 0, 0, 0),
                "box", {"size": "0.41 0.022 0.018"}, graphite)
        for end, x in (("front", 0.12), ("rear", -0.12)):
            _visual(link, f"skid_strut_{side}_{end}", (center_x + x, center_y + y, -0.045, 0, 0, 0),
                    "cylinder", {"radius": "0.009", "length": "0.085"}, aluminum)


def write_world(path: Path, model: dict) -> None:
    """Write the standalone display-only world. SITL owns the drone pose."""
    scene = ET.fromstring(_SCENE)
    ground = scene.find("./world/model[@name='test_field']/link")
    airframe = scene.find(f"./world/model[@name='{MODEL_NAME}']/link")
    _add_field(ground)
    _add_airframe(airframe, model)
    ET.indent(scene)
    ET.ElementTree(scene).write(path, encoding="utf-8", xml_declaration=True)
