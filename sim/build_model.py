"""Build the Gazebo (Harmonic) model of the Ster-Vis test AMR from its CAD.

The design lives in the Ster-Vis repo, in hardware/test-amr/ (branch
feat/test-amr): chassis.py is the Fusion 360 generator. This script:

- reads the dimensions straight out of chassis.py, so an edit there carries
  over here on the next run;
- takes the printed parts in their assembled positions (cad/assembly, which
  chassis.py exports) and writes them, in metres, to
  models/ster_vis_test_amr/meshes/;
- adds the bought parts (motors, wheels, Pi 5, L298Ns, battery, buck converter,
  cameras) as boxes, sized as the stand-ins in chassis.py;
- writes models/ster_vis_test_amr/model.sdf: 4WD skid steer on the stock 65 mm
  wheels, and two Camera Module 3 (66 deg, 1280x720) on the 60 mm baseline;
- writes two worlds: worlds/amr_test.sdf (targets at 1, 2 and 3 m) and
  worlds/amr_arena.sdf (a walled arena with obstacles, for amr run --sim).

    python build_model.py                       # finds ../Stereo-vision or ../Ster-Vis
    python build_model.py --source PATH/TO/hardware/test-amr

Frames: the model's origin is on the floor under the centre of the chassis,
x forward, y left, z up (base_link, as in the chassis README). chassis.py puts
z = 0 at the underside of the deck, about 43 mm above the floor on the stock
wheels.
"""

from __future__ import annotations

import argparse
import ast
import json
import math
import struct
from pathlib import Path

import numpy as np

HERE = Path(__file__).parent
MODEL_NAME = "ster_vis_test_amr"
MODEL_DIR = HERE / "models" / MODEL_NAME
MESH_DIR = MODEL_DIR / "meshes"
WORLD_DIR = HERE / "worlds"
SOURCE_CANDIDATES = [HERE.parent.parent / "Stereo-vision" / "hardware" / "test-amr",
                     HERE.parent.parent / "Ster-Vis" / "hardware" / "test-amr"]

# Camera Module 3, standard lens: 66 deg horizontal. Ster-Vis captures
# 1280x720 by default, which keeps the sensor's 16:9.
CAM_WIDTH, CAM_HEIGHT, CAM_HFOV, CAM_RATE = 1280, 720, math.radians(66.0), 10
LENS_LEN = 5.0            # mm; the lens stand-in in chassis.py's camera()

# Masses (kg) of the bought parts: typical published figures, not weighed.
MOTOR_KG, WHEEL_KG = 0.030, 0.028
BATTERY_KG = 0.175         # Bonka 3S 2200 mAh 35C, per its listing
PLA_DENSITY, ABS_DENSITY = 1.24e-6, 1.04e-6   # kg per mm^3
PRINT_FILL = 0.75         # 4 perimeters, 5 top/bottom layers, 30% infill: thin parts are mostly solid

# BO motor at about 200 rpm on a 65 mm wheel is about 0.68 m/s; the 55% PWM cap
# on 3S keeps it near 6 V. Limits a little under that.
MAX_SPEED, MAX_TURN, MAX_ACCEL = 0.6, 3.0, 1.0

GREY_PLA = "0.62 0.63 0.66"
LIGHT_PLA = "0.85 0.85 0.87"      # the printed pins, wedges, clips and covers
DARK_ABS = "0.22 0.22 0.24"
YELLOW = "0.95 0.75 0.10"
GREEN = "0.10 0.45 0.20"
RED = "0.70 0.12 0.10"
BLUE = "0.15 0.20 0.65"
BLACK = "0.05 0.05 0.05"
ALU = "0.75 0.75 0.78"


# -- dimensions, read from chassis.py ------------------------------------------

def load_dimensions(chassis_py: Path) -> dict:
    """Run chassis.py's module-level constants and helper functions, skipping
    everything that needs Fusion (the adsk imports and the Builder class)."""
    tree = ast.parse(chassis_py.read_text(encoding="utf-8"))
    ns: dict = {"math": math}
    for node in tree.body:
        if isinstance(node, ast.Assign) or (isinstance(node, ast.FunctionDef)
                                            and node.name in ("pi_holes", "cam_holes", "lens_z")):
            exec(compile(ast.Module([node], []), str(chassis_py), "exec"), ns)
    return ns


# -- STL in and out ---------------------------------------------------------------

def read_stl(path: Path) -> np.ndarray:
    """Binary STL -> (n, 3, 3) float64 triangles."""
    data = path.read_bytes()
    count = struct.unpack("<I", data[80:84])[0]
    if len(data) != 84 + 50 * count:
        raise ValueError(f"{path} is not a binary STL")
    rec = np.frombuffer(data[84:], np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]))
    return rec["v"].astype(np.float64)


def write_stl(path: Path, tris: np.ndarray, name: str) -> None:
    normals = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    normals /= np.maximum(np.linalg.norm(normals, axis=1, keepdims=True), 1e-12)
    rec = np.zeros(len(tris), np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]))
    rec["n"], rec["v"] = normals, tris
    header = f"{name} (metres, chassis.py frame)".encode()[:80].ljust(80, b" ")
    path.write_bytes(header + struct.pack("<I", len(tris)) + rec.tobytes())


def volume_mm3(tris: np.ndarray) -> float:
    return float(abs(np.einsum("ij,ij->i", tris[:, 0], np.cross(tris[:, 1], tris[:, 2])).sum()) / 6.0)


# -- SDF snippets -----------------------------------------------------------------

def material(rgb: str) -> str:
    return (f"<material><ambient>{rgb} 1</ambient><diffuse>{rgb} 1</diffuse>"
            f"<specular>0.15 0.15 0.15 1</specular></material>")


def pose(x: float, y: float, z: float, roll: float = 0.0, pitch: float = 0.0, yaw: float = 0.0) -> str:
    return f"<pose>{x:.5f} {y:.5f} {z:.5f} {roll:.5f} {pitch:.5f} {yaw:.5f}</pose>"


class Chassis:
    """Everything bolted to the chassis: one rigid link, base_link."""

    def __init__(self, z0_mm: float):
        self.z0 = z0_mm
        self.visuals: list[str] = []
        self.collisions: list[str] = []
        self.masses: list[tuple[float, np.ndarray, np.ndarray]] = []   # kg, centre (m), size (m)

    def m(self, x: float, y: float, z: float) -> tuple[float, float, float]:
        """chassis.py mm -> model-frame metres."""
        return x / 1000, y / 1000, (z + self.z0) / 1000

    def box(self, name, x0, x1, y0, y1, z0, z1, rgb=None, kg=0.0, collide=False):
        x0, x1 = sorted((x0, x1))
        y0, y1 = sorted((y0, y1))
        z0, z1 = sorted((z0, z1))
        centre = self.m((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2)
        size = ((x1 - x0) / 1000, (y1 - y0) / 1000, (z1 - z0) / 1000)
        geom = f"<box><size>{size[0]:.5f} {size[1]:.5f} {size[2]:.5f}</size></box>"
        if rgb:
            self.visuals.append(f'<visual name="{name}">{pose(*centre)}<geometry>{geom}</geometry>{material(rgb)}</visual>')
        if collide:
            self.collisions.append(f'<collision name="{name}">{pose(*centre)}<geometry>{geom}</geometry></collision>')
        if kg:
            self.masses.append((kg, np.array(centre), np.array(size)))

    def mesh(self, name, uri, rgb, at=(0.0, 0.0, 0.0)):
        self.visuals.append(f'<visual name="{name}">{pose(*self.m(*at))}'
                            f"<geometry><mesh><uri>{uri}</uri></mesh></geometry>{material(rgb)}</visual>")

    def inertial(self) -> str:
        total = sum(kg for kg, _, _ in self.masses)
        com = sum(kg * c for kg, c, _ in self.masses) / total
        inertia = np.zeros((3, 3))
        for kg, c, (a, b, h) in self.masses:
            inertia += np.diag([kg * (b * b + h * h), kg * (a * a + h * h), kg * (a * a + b * b)]) / 12
            d = c - com
            inertia += kg * (d @ d * np.eye(3) - np.outer(d, d))
        return (f"<inertial>{pose(*com)}<mass>{total:.4f}</mass><inertia>"
                f"<ixx>{inertia[0, 0]:.3e}</ixx><ixy>{inertia[0, 1]:.3e}</ixy><ixz>{inertia[0, 2]:.3e}</ixz>"
                f"<iyy>{inertia[1, 1]:.3e}</iyy><iyz>{inertia[1, 2]:.3e}</iyz><izz>{inertia[2, 2]:.3e}</izz>"
                f"</inertia></inertial>")


def wheel_link(name: str, x: float, y: float, z: float, radius: float, width: float) -> str:
    ixx = WHEEL_KG * (3 * radius ** 2 + width ** 2) / 12
    izz = WHEEL_KG * radius ** 2 / 2
    tyre = f"<cylinder><radius>{radius:.5f}</radius><length>{width:.5f}</length></cylinder>"
    hub = f"<cylinder><radius>{radius * 0.55:.5f}</radius><length>{width + 0.001:.5f}</length></cylinder>"
    # a bar across the hub, so the renders show the wheel turning
    spoke = f"<box><size>{radius * 1.1:.5f} {radius * 0.18:.5f} {width + 0.002:.5f}</size></box>"
    return f"""
    <link name="wheel_{name}">
      {pose(x, y, z, -math.pi / 2)}
      <inertial><mass>{WHEEL_KG}</mass>
        <inertia><ixx>{ixx:.3e}</ixx><iyy>{ixx:.3e}</iyy><izz>{izz:.3e}</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
      </inertial>
      <collision name="collision">
        <geometry>{tyre}</geometry>
        <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface>
      </collision>
      <visual name="tyre"><geometry>{tyre}</geometry>{material(BLACK)}</visual>
      <visual name="hub"><geometry>{hub}</geometry>{material(YELLOW)}</visual>
      <visual name="spoke"><geometry>{spoke}</geometry>{material(ALU)}</visual>
    </link>
    <joint name="wheel_{name}_joint" type="revolute">
      <parent>base_link</parent><child>wheel_{name}</child>
      <axis><xyz>0 0 1</xyz><limit><lower>-1e16</lower><upper>1e16</upper></limit>
        <dynamics><damping>0.0005</damping></dynamics></axis>
    </joint>"""


def camera_sensor(side: str, x: float, y: float, z: float) -> str:
    return f"""
      <sensor name="{side}_camera" type="camera">
        {pose(x, y, z)}
        <always_on>1</always_on>
        <update_rate>{CAM_RATE}</update_rate>
        <topic>/amr/stereo/{side}/image</topic>
        <camera name="{side}">
          <horizontal_fov>{CAM_HFOV:.5f}</horizontal_fov>
          <image><width>{CAM_WIDTH}</width><height>{CAM_HEIGHT}</height><format>R8G8B8</format></image>
          <clip><near>0.02</near><far>30</far></clip>
          <noise><type>gaussian</type><mean>0</mean><stddev>0.005</stddev></noise>
        </camera>
      </sensor>"""


# -- the model -----------------------------------------------------------------------

def build_model(source: Path) -> dict:
    d = load_dimensions(source / "chassis.py")
    MESH_DIR.mkdir(parents=True, exist_ok=True)
    uri = "meshes"                  # relative to model.sdf; gz resolves it from the model folder

    z0 = -d["FLOOR_Z"]                                  # floor to the underside of the deck
    x0_deck, x1_deck, deck_w, z_top = d["DECK_X0"], d["DECK_X1"], d["DECK_W"], d["Z_TOP"]
    ch = Chassis(z0)

    # Printed parts: chassis.py exports them where they sit on the robot
    # (cad/assembly, mm, its own frame), so they only need scaling to metres.
    for stale in MESH_DIR.glob("*.stl"):
        stale.unlink()
    assembly = source / "cad" / "assembly"
    parts = {"deck": (GREY_PLA, PLA_DENSITY), "camera_clamps": (LIGHT_PLA, PLA_DENSITY)}
    printed_kg = {}
    for name, (rgb, density) in parts.items():
        tris = read_stl(assembly / f"{name}.stl")
        lo, hi = tris.reshape(-1, 3).min(0), tris.reshape(-1, 3).max(0)
        printed_kg[name] = volume_mm3(tris) * density * PRINT_FILL
        write_stl(MESH_DIR / f"{name}.stl", tris / 1000, name)      # chassis.py frame; the visual pose lifts it
        ch.mesh(name, f"{uri}/{name}.stl", rgb)
        ch.box(f"{name}_mass", lo[0], hi[0], lo[1], hi[1], lo[2], hi[2], kg=printed_kg[name])

    # Bought parts, as the stand-ins in chassis.py.
    inner = d["MOTOR_OUTER_Y"] - d["TT_THICK"]
    zc, top = d["SHAFT_Z"], d["POCKET"]
    for s, sname in ((1, "front"), (-1, "rear")):
        for t, tname in ((1, "left"), (-1, "right")):
            sx = s * d["MOTOR_X"]
            ch.box(f"motor_{sname}_{tname}_gearbox", sx + s * d["TT_FRONT"], sx + s * (d["TT_FRONT"] - d["TT_GEARBOX"]),
                   t * inner, t * d["MOTOR_OUTER_Y"], top - d["TT_HEIGHT"], top, YELLOW, MOTOR_KG * 0.6)
            ch.box(f"motor_{sname}_{tname}_can", sx + s * (d["TT_FRONT"] - d["TT_GEARBOX"]),
                   sx + s * (d["TT_FRONT"] - d["TT_LENGTH"]), t * (inner + 1.9), t * (d["MOTOR_OUTER_Y"] - 1.9),
                   zc - 10, zc + 10, ALU, MOTOR_KG * 0.4)
    pz = z_top + d["PI_POST_H"]
    pi_x, sd_y = d["PI_X"], d["PI_SD_Y"]
    ports_y = sd_y - 85
    ch.box("pi5_board", pi_x - 28, pi_x + 28, ports_y, sd_y, pz, pz + 1.6, GREEN, 0.035)
    ch.box("pi5_ports", pi_x - 26, pi_x + 26, ports_y - 2, ports_y + 22, pz + 1.6, pz + 17.6, ALU, 0.011)
    ch.box("pi5_cooler", pi_x - 22, pi_x + 22, ports_y + 26, sd_y - 8, pz + 1.6, pz + 32, ALU, 0.020)
    lz = z_top + d["L298N_POST_H"]
    half = d["L298N_BOARD"] / 2
    for cx, cy in d["L298N_CENTRES"]:
        side = "left" if cy > 0 else "right"
        ch.box(f"l298n_{side}_board", cx - half, cx + half, cy - half, cy + half, lz, lz + 1.6, RED, 0.012)
        ch.box(f"l298n_{side}_heatsink", cx - 11.5, cx + 11.5, cy - 8, cy + 8, lz + 1.6, lz + 27, BLACK, 0.016)
    bx0, bx1, bhl, bh = d["BATTERY"]
    ch.box("battery_bonka_3s", bx0, bx1, -bhl, bhl, z_top, z_top + bh, BLUE, BATTERY_KG)
    ux0, ux1, uy0, uy1 = d["BUCK"]
    ch.box("buck_5v", ux0, ux1, uy0, uy1, z_top, z_top + d["BUCK_H"], GREEN, 0.020)
    lens_z = d["lens_z"]()
    x_board = d["X_BOARD"]
    lens_x = x_board + d["CAM_BOARD_T"] + LENS_LEN
    for side, cy in (("left", d["BASELINE"] / 2), ("right", -d["BASELINE"] / 2)):
        ch.box(f"camera_{side}_board", x_board, x_board + d["CAM_BOARD_T"], cy - d["CAM_BOARD_W"] / 2,
               cy + d["CAM_BOARD_W"] / 2, d["CAM_BOARD_Z0"], d["CAM_BOARD_Z0"] + d["CAM_BOARD_H"], GREEN, 0.003)
        ch.box(f"camera_{side}_lens", x_board + d["CAM_BOARD_T"], lens_x, cy - 4.25, cy + 4.25,
               lens_z - 4.25, lens_z + 4.25, BLACK, 0.001)

    # Collision: a few boxes around the real outline, no meshes.
    ch.box("c_deck", x0_deck, x1_deck, -deck_w / 2, deck_w / 2, 0, z_top, collide=True)
    motor_x = d["MOTOR_X"] + d["TT_FRONT"]
    for t, tname in ((1, "left"), (-1, "right")):
        ch.box(f"c_motors_{tname}", -motor_x, motor_x, t * inner, t * d["MOTOR_OUTER_Y"], top - d["TT_HEIGHT"], 0,
               collide=True)
    ch.box("c_battery", bx0 - 2.5, bx1 + 2.5, -bhl - 2.5, bhl + 2.5, z_top, z_top + bh, collide=True)
    ch.box("c_boards", -45, pi_x + 28, ports_y - 2, sd_y, z_top, pz + 32, collide=True)
    ch.box("c_camera_bracket", d["PLATE_X"], lens_x, -d["PLATE_HALF_W"], d["PLATE_HALF_W"], z_top,
           d["PLATE_TOP"], collide=True)

    # Wheels on the motor shafts.
    radius, width = d["WHEEL_D"] / 2000, d["WHEEL_W"] / 1000
    wheel_y = (d["MOTOR_OUTER_Y"] + d["WHEEL_GAP"] + d["WHEEL_W"] / 2) / 1000
    wheels = "".join(wheel_link(f"{sname}_{tname}", s * d["MOTOR_X"] / 1000, t * wheel_y, radius, radius, width)
                     for s, sname in ((1, "front"), (-1, "rear")) for t, tname in ((1, "left"), (-1, "right")))

    # Cameras: the sensor sits just in front of the lens stand-in, so it does
    # not see the inside of it.
    cam_x, cam_z = (lens_x + 0.5) / 1000, (lens_z + z0) / 1000
    half_b = d["BASELINE"] / 2000

    sdf = f"""<?xml version="1.0" ?>
<!-- GENERATED by build_model.py from the Ster-Vis hardware/test-amr CAD - edit
     that (or build_model.py), not this. Origin: on the floor under the centre
     of the wheelbase; x forward, y left, z up. -->
<sdf version="1.10">
  <model name="{MODEL_NAME}">
    <link name="base_link">
      {ch.inertial()}
      {"".join(ch.collisions)}
      {"".join(ch.visuals)}
      <!-- The Ster-Vis stereo pair: two Camera Module 3, {d["BASELINE"]:.0f} mm apart. -->
      {camera_sensor("left", cam_x, half_b, cam_z)}
      {camera_sensor("right", cam_x, -half_b, cam_z)}
    </link>
    {wheels}
    <!-- 4WD skid steer: both motors on a side run together, as each L298N
         drives one side. The odometry is perfect wheel odometry, which the
         real robot does not have (its BO motors have no encoders). -->
    <plugin filename="gz-sim-diff-drive-system" name="gz::sim::systems::DiffDrive">
      <left_joint>wheel_front_left_joint</left_joint><left_joint>wheel_rear_left_joint</left_joint>
      <right_joint>wheel_front_right_joint</right_joint><right_joint>wheel_rear_right_joint</right_joint>
      <wheel_separation>{2 * wheel_y:.4f}</wheel_separation>
      <wheel_radius>{radius:.5f}</wheel_radius>
      <max_linear_velocity>{MAX_SPEED}</max_linear_velocity>
      <min_linear_velocity>{-MAX_SPEED}</min_linear_velocity>
      <max_angular_velocity>{MAX_TURN}</max_angular_velocity>
      <min_angular_velocity>{-MAX_TURN}</min_angular_velocity>
      <max_linear_acceleration>{MAX_ACCEL}</max_linear_acceleration>
      <min_linear_acceleration>{-MAX_ACCEL}</min_linear_acceleration>
      <topic>/amr/cmd_vel</topic>
      <odom_topic>/amr/odometry</odom_topic>
      <tf_topic>/amr/tf</tf_topic>
      <frame_id>odom</frame_id>
      <child_frame_id>base_link</child_frame_id>
      <odom_publish_frequency>30</odom_publish_frequency>
    </plugin>
    <plugin filename="gz-sim-joint-state-publisher-system" name="gz::sim::systems::JointStatePublisher">
      <topic>/amr/joint_states</topic>
    </plugin>
  </model>
</sdf>
"""
    (MODEL_DIR / "model.sdf").write_text(sdf, encoding="utf-8")
    (MODEL_DIR / "model.config").write_text(f"""<?xml version="1.0" ?>
<model>
  <name>Ster-Vis test AMR</name>
  <version>1.0</version>
  <sdf version="1.10">model.sdf</sdf>
  <description>
    4WD skid-steer test robot for Ster-Vis: one 3D-printed deck, BO (TT) gear
    motors on 65 mm wheels, Raspberry Pi 5, two L298N drivers, 3S LiPo, and two
    Camera Module 3 on a {d["BASELINE"]:.0f} mm baseline. Generated from
    hardware/test-amr/chassis.py by build_model.py.
  </description>
</model>
""", encoding="utf-8")

    total = sum(kg for kg, _, _ in ch.masses) + 4 * WHEEL_KG
    return {
        "mass_kg": round(total, 3),
        "printed_kg": {k: round(v, 3) for k, v in printed_kg.items()},
        "wheelbase_m": 2 * d["MOTOR_X"] / 1000,
        "track_m": round(2 * wheel_y, 4),
        "wheel_radius_m": radius,
        "left_camera_m": (round(cam_x, 4), round(half_b, 4), round(cam_z, 4)),
        "ground_clearance_mm": round(z0 + top - d["TT_HEIGHT"], 2),
    }


# -- the test world -------------------------------------------------------------------

def noise_texture(path: Path, tint: tuple[int, int, int], seed: int, size: int = 512) -> None:
    """Multi-scale noise, as in Ster-Vis's sim/gazebo/make_textures.py: stereo
    needs texture to match."""
    import cv2

    rng = np.random.default_rng(seed)
    image = np.zeros((size, size), np.float32)
    for cells, weight in ((8, 0.35), (32, 0.3), (128, 0.2), (size, 0.15)):
        image += weight * cv2.resize(rng.random((cells, cells), dtype=np.float32), (size, size),
                                     interpolation=cv2.INTER_CUBIC)
    image = cv2.normalize(image, None, 0.15, 1.0, cv2.NORM_MINMAX)
    cv2.imwrite(str(path), np.clip(np.dstack([image * c for c in tint[::-1]]), 0, 255).astype(np.uint8))


# Test targets in front of the robot: (name, x, y, size x, size y, height, texture).
TARGETS = [
    ("target_1m", 1.0, 0.0, 0.10, 0.30, 0.30, "target_red.png"),
    ("target_2m", 2.0, 0.45, 0.10, 0.40, 0.45, "target_yellow.png"),
    ("wall_3m", 3.0, 0.0, 0.10, 3.0, 0.8, "wall.png"),
]


def textured(rgb_texture: str) -> str:
    return f"""<material>
          <ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse><specular>0.05 0.05 0.05 1</specular>
          <pbr><metal><albedo_map>textures/{rgb_texture}</albedo_map><roughness>0.9</roughness><metalness>0</metalness></metal></pbr>
        </material>"""


def view_camera(name: str, eye: tuple[float, float, float], look: tuple[float, float, float], hfov: float,
                size: tuple[int, int] = (1280, 800)) -> str:
    """A fixed camera in the world, not on the robot, for looking at it."""
    dx, dy, dz = (look[i] - eye[i] for i in range(3))
    yaw, pitch = math.atan2(dy, dx), -math.atan2(dz, math.hypot(dx, dy))
    return f"""
    <model name="{name}">
      <static>true</static>
      {pose(*eye, 0.0, pitch, yaw)}
      <link name="link">
        <sensor name="camera" type="camera">
          <always_on>1</always_on><update_rate>5</update_rate>
          <topic>/view/{name}/image</topic>
          <camera><horizontal_fov>{hfov}</horizontal_fov>
            <image><width>{size[0]}</width><height>{size[1]}</height><format>R8G8B8</format></image>
            <clip><near>0.02</near><far>30</far></clip></camera>
        </sensor>
      </link>
    </model>"""


def box_model(name: str, x: float, y: float, sx: float, sy: float, h: float, texture: str, yaw: float = 0.0) -> str:
    return f"""
    <model name="{name}">
      <static>true</static>
      {pose(x, y, h / 2, 0, 0, yaw)}
      <link name="link">
        <collision name="collision"><geometry><box><size>{sx:.3f} {sy:.3f} {h:.3f}</size></box></geometry></collision>
        <visual name="visual"><geometry><box><size>{sx:.3f} {sy:.3f} {h:.3f}</size></box></geometry>
        {textured(texture)}
        </visual>
      </link>
    </model>"""


def cylinder_model(name: str, x: float, y: float, r: float, h: float, texture: str) -> str:
    geom = f"<cylinder><radius>{r:.3f}</radius><length>{h:.3f}</length></cylinder>"
    return f"""
    <model name="{name}">
      <static>true</static>
      {pose(x, y, h / 2)}
      <link name="link">
        <collision name="collision"><geometry>{geom}</geometry></collision>
        <visual name="visual"><geometry>{geom}</geometry>
        {textured(texture)}
        </visual>
      </link>
    </model>"""


def floor_tiles(x0: float, x1: float, y0: float, y1: float, tile: float = 1.0) -> str:
    """Textured floor in tiles, so the texture keeps its scale (visual only).
    Stereo needs it: a flat floor gives false matches near the horizon."""
    out = []
    for i in range(int(round((x1 - x0) / tile))):
        for j in range(int(round((y1 - y0) / tile))):
            out.append(f"""
    <model name="floor_{i}_{j}">
      <static>true</static>
      <pose>{x0 + tile * (i + 0.5):.2f} {y0 + tile * (j + 0.5):.2f} 0.0005 0 0 0</pose>
      <link name="link"><visual name="visual">
        <geometry><plane><normal>0 0 1</normal><size>{tile} {tile}</size></plane></geometry>
        {textured("floor.png")}
      </visual></link>
    </model>""")
    return "".join(out)


def make_textures() -> None:
    tex = WORLD_DIR / "textures"
    tex.mkdir(parents=True, exist_ok=True)
    for name, tint, seed in (("floor.png", (150, 130, 110), 5), ("target_red.png", (230, 90, 70), 1),
                             ("target_yellow.png", (230, 200, 80), 2), ("target_green.png", (90, 200, 110), 3),
                             ("wall.png", (200, 200, 210), 4)):
        noise_texture(tex / name, tint, seed)


def world_sdf(name: str, comment: str, body: str, robot_pose: tuple[float, float, float]) -> str:
    x, y, yaw = robot_pose
    return f"""<?xml version="1.0" ?>
<!-- GENERATED by build_model.py - edit that, not this.
     {comment} -->
<sdf version="1.10">
  <world name="{name}">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>

    <scene>
      <ambient>0.6 0.6 0.6 1</ambient>
      <background>0.7 0.8 0.9 1</background>
      <shadows>false</shadows>
    </scene>
    <light type="directional" name="sun">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.1 0.1 0.1 1</specular>
      <direction>-0.4 0.2 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision"><geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry></collision>
        <visual name="visual"><geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
          <material><ambient>0.55 0.5 0.45 1</ambient><diffuse>0.55 0.5 0.45 1</diffuse></material></visual>
      </link>
    </model>
{body}

    <include>
      <uri>../models/{MODEL_NAME}</uri>
      <name>amr</name>
      <pose>{x:.3f} {y:.3f} 0.002 0 0 {yaw:.4f}</pose>
    </include>
  </world>
</sdf>
"""


def build_world() -> None:
    """amr_test: targets at 1, 2 and 3 m, and cameras to look at the robot."""
    targets = "".join(box_model(name, x, y, sx, sy, h, texture) for name, x, y, sx, sy, h, texture in TARGETS)
    # Not part of the robot: fixed cameras to look at it. The first matches
    # the chassis README's render.
    views = (view_camera("view_iso", (0.36, 0.30, 0.28), (0.0, 0.0, 0.065), 0.75)
             + view_camera("view_side", (0.0, -0.75, 0.12), (0.0, 0.0, 0.07), 0.6)
             + view_camera("view_scene", (-1.2, -1.1, 1.1), (1.2, 0.2, 0.0), 1.2))
    body = floor_tiles(-3.0, 5.0, -4.0, 4.0) + targets + views
    (WORLD_DIR / "amr_test.sdf").write_text(
        world_sdf("amr_test", "Ster-Vis test AMR on a textured floor, with targets at 1, 2 and 3 m.", body,
                  (0.0, 0.0, 0.0)), encoding="utf-8")


ARENA = 4.0                               # square side, metres
ARENA_WALL_H = 0.35
ARENA_START = (-1.4, -1.4, math.radians(45))
ROBOT_GAP = 0.45                          # kept free between obstacles: the robot is 0.2 m wide


def build_arena(seed: int = 3, count: int = 10) -> None:
    """amr_arena: a 4 x 4 m walled arena with obstacles, for `amr run --sim`.

    Obstacles are 0.15-0.4 m tall, so all of them reach into the scan band
    (3-17 cm above the floor). Every shape also goes to amr_arena.json, so a
    run can be scored against the truth (score_run.py).
    """
    rng = np.random.default_rng(seed)
    half = ARENA / 2
    body, truth = [], []
    segment = 1.0
    for side, (fixed, horizontal) in enumerate(((half, True), (-half, True), (half, False), (-half, False))):
        for i in range(int(ARENA / segment)):
            along = -half + segment * (i + 0.5)
            x, y = (along, fixed) if horizontal else (fixed, along)
            yaw = 0.0 if horizontal else math.pi / 2
            name = f"wall_{side}_{i}"
            body.append(box_model(name, x, y, segment, 0.05, ARENA_WALL_H, "wall.png", yaw))
            truth.append({"name": name, "kind": "box", "x": x, "y": y, "sx": segment, "sy": 0.05, "yaw": yaw})
    textures = ("target_red.png", "target_yellow.png", "target_green.png")
    placed = []
    while len(placed) < count:
        x, y = (float(v) for v in rng.uniform(-half + 0.45, half - 0.45, 2))
        if math.hypot(x - ARENA_START[0], y - ARENA_START[1]) < 0.7:
            continue                     # keep the start clear
        h = float(rng.uniform(0.15, 0.4))
        if rng.random() < 0.6:
            sx, sy = (float(v) for v in rng.uniform(0.12, 0.35, 2))
            radius = math.hypot(sx, sy) / 2
            shape = {"kind": "box", "sx": round(sx, 4), "sy": round(sy, 4), "yaw": round(float(rng.uniform(0, math.pi)), 4)}
        else:
            radius = float(rng.uniform(0.05, 0.15))
            shape = {"kind": "cylinder", "r": round(radius, 4)}
        if any(math.hypot(x - px, y - py) < radius + pr + ROBOT_GAP for px, py, pr in placed):
            continue
        name = f"obstacle_{len(placed)}"
        placed.append((x, y, radius))
        texture = textures[len(placed) % 3]
        if shape["kind"] == "box":
            body.append(box_model(name, x, y, shape["sx"], shape["sy"], h, texture, shape["yaw"]))
        else:
            body.append(cylinder_model(name, x, y, shape["r"], h, texture))
        truth.append({"name": name, "x": round(x, 4), "y": round(y, 4), "h": round(h, 3), **shape})
    view = view_camera("view_top", (0.0, -0.9, 6.2), (0.0, 0.0, 0.0), 0.8, (1000, 1000))
    body_sdf = floor_tiles(-half, half, -half, half) + "".join(body) + view
    comment = f"{ARENA:.0f} x {ARENA:.0f} m arena with {count} obstacles (seed {seed}), for amr run --sim."
    (WORLD_DIR / "amr_arena.sdf").write_text(world_sdf("amr_arena", comment, body_sdf, ARENA_START), encoding="utf-8")
    (WORLD_DIR / "amr_arena.json").write_text(json.dumps(
        {"arena": ARENA, "start": ARENA_START, "obstacles": truth}, indent=1), encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--source", type=Path, help="the Ster-Vis hardware/test-amr folder")
    args = parser.parse_args()
    source = args.source or next((p for p in SOURCE_CANDIDATES if (p / "chassis.py").exists()), None)
    if source is not None and (source / "chassis.py").exists():
        summary = build_model(source)
        print(f"model: {MODEL_DIR}")
        for key, value in summary.items():
            print(f"  {key}: {value}")
    elif (MODEL_DIR / "model.sdf").exists():
        print("hardware/test-amr not found (Ster-Vis branch feat/test-amr, or --source): kept the model as it is")
    else:
        raise SystemExit("hardware/test-amr not found: check out the Ster-Vis branch feat/test-amr, "
                         "or pass --source")
    make_textures()
    build_world()
    build_arena()
    print(f"worlds: {WORLD_DIR / 'amr_test.sdf'}, {WORLD_DIR / 'amr_arena.sdf'}")


if __name__ == "__main__":
    main()
