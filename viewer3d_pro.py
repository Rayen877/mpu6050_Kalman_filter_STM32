"""RViz-style 3D comparison of three IMU boards: raw accelerometer | complementary | EKF.

Install : pip install -U pyvista numpy pyserial
Run     : python viewer3d_pro.py COM5        (real data from the STM32)
          python viewer3d_pro.py demo        (fake motion, to test the look without the board)
Input   : 6 fields at 115200 baud (firmware with the 6-field CSV line):
          accRoll,accPitch,ekfRoll,ekfPitch,compRoll,compPitch
Note    : close PuTTY / viewer.py / MATLAB first (only one program can use the COM port).

Frame convention (same as ROS / RViz): X = red = front, Y = green = left, Z = blue = up.
Mouse: left drag = rotate, wheel = zoom, middle drag = pan.
"""
import sys
import time

import numpy as np
import pyvista as pv

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM5"
BAUD = 115200
SPACING = 9.0        # distance between boards
UPDATE_MS = 15       # refresh period of the 3D view

# The red arrow on each model is the sensor X axis (roll = rotation about it), the green one is Y (pitch).
# BOARD_ALONG_Y = True : the long side of the drawn board is along Y (X crosses the short side)
# BOARD_ALONG_Y = False: the long side of the drawn board is along X
# Pick the value for which the red arrow matches the "X" printed on your MPU6050 chip.
BOARD_ALONG_Y = True

# Titles are shifted to the left (screen) by this much per character, to sit centered above each board.
# Bigger value = more to the left, 0 = old position.
TITLE_SHIFT_PER_CHAR = 0.125

# name, y position (raw on the left of the screen, EKF on the right), color
MODELS = {
    "raw":  ("RAW ACCELEROMETER", +SPACING, "#ff9f1c"),
    "comp": ("COMPLEMENTARY",      0.0,     "#2ec4b6"),
    "ekf":  ("Extended Kalman Filter",  -SPACING, "#3a86ff"),
}


# ---------------------------------------------------------------- data sources
class SerialSource:
    def __init__(self, port):
        import serial
        self.ser = serial.Serial(port, BAUD, timeout=0)
        self.ser.reset_input_buffer()
        self.buf = b""

    def latest(self):
        n = self.ser.in_waiting
        if n:
            self.buf += self.ser.read(n)
        *lines, self.buf = self.buf.split(b"\n")
        last = None
        for line in lines:
            parts = line.decode(errors="ignore").strip().split(",")
            if len(parts) != 6:
                continue
            try:
                last = [float(p) for p in parts]
            except ValueError:
                continue
        return last

    def close(self):
        self.ser.close()


class DemoSource:
    """Fake motion: the raw board is noisy, the filtered ones are smooth."""
    def __init__(self):
        self.t0 = time.time()
        self.rng = np.random.default_rng(1)

    def latest(self):
        t = time.time() - self.t0
        roll = 40 * np.sin(0.9 * t)
        pitch = 25 * np.sin(0.6 * t + 1.0)
        n = lambda s: self.rng.normal(0, s)
        return [roll + n(4), pitch + n(4),
                roll + n(0.2), pitch + n(0.2),
                roll + n(0.6), pitch + n(0.6)]

    def close(self):
        pass


# ---------------------------------------------------------------- geometry
def rot(roll_deg, pitch_deg):
    """Body -> world rotation, roll about X then pitch about Y (yaw = 0)."""
    r, p = np.radians(roll_deg), np.radians(pitch_deg)
    cr, sr, cp, sp = np.cos(r), np.sin(r), np.cos(p), np.sin(p)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    return ry @ rx


def board_parts(color):
    if BOARD_ALONG_Y:
        pcb = pv.Cube(center=(0, 0, 0), x_length=3.0, y_length=4.0, z_length=0.3)
        chip = pv.Cube(center=(0, 0.5, 0.25), x_length=1.3, y_length=1.3, z_length=0.25)
        pins = pv.Cube(center=(0, -1.7, 0.22), x_length=2.6, y_length=0.5, z_length=0.3)
        nose = pv.Cube(center=(1.35, 0, 0.17), x_length=0.3, y_length=1.0, z_length=0.05)
    else:
        pcb = pv.Cube(center=(0, 0, 0), x_length=4.0, y_length=3.0, z_length=0.3)
        chip = pv.Cube(center=(0.5, 0, 0.25), x_length=1.3, y_length=1.3, z_length=0.25)
        pins = pv.Cube(center=(-1.7, 0, 0.22), x_length=0.5, y_length=2.6, z_length=0.3)
        nose = pv.Cube(center=(1.85, 0, 0.17), x_length=0.3, y_length=1.0, z_length=0.05)
    return [(pcb, color, False), (chip, "#14161b", False),
            (pins, "#d4af37", False), (nose, "#f1f1f1", False)]


def frame_parts(length=3.6):
    parts = []
    for d, c in (((1, 0, 0), "#e63946"), ((0, 1, 0), "#57cc99"), ((0, 0, 1), "#4895ef")):
        a = pv.Arrow(start=(0, 0, 0), direction=d, scale=length,
                     tip_length=0.22, tip_radius=0.07, shaft_radius=0.03)
        parts.append((a, c, True))
    return parts


def set_pose(actors, pos, roll, pitch):
    m = np.eye(4)
    m[:3, :3] = rot(roll, pitch)
    m[:3, 3] = pos
    for a in actors:
        a.user_matrix = m


# ---------------------------------------------------------------- scene
def build_scene(pl):
    pl.set_background("#1b1d23", top="#3b4252")          # soft dark gradient

    floor_z = -3.8
    pl.add_mesh(pv.Plane(center=(0, 0, floor_z - 0.05), direction=(0, 0, 1),
                         i_size=60, j_size=60, i_resolution=1, j_resolution=1),
                color="#23262d", smooth_shading=True)
    pl.add_mesh(pv.Plane(center=(0, 0, floor_z), direction=(0, 0, 1),
                         i_size=60, j_size=60, i_resolution=30, j_resolution=30),
                style="wireframe", color="#555b69", line_width=1, opacity=0.55)

    # world frame at the origin of the floor (like the TF display in RViz)
    for part, col, smooth in frame_parts(4.0):
        a = pl.add_mesh(part, color=col, smooth_shading=smooth, ambient=0.2, diffuse=0.8)
        a.user_matrix = np.array([[1, 0, 0, 14], [0, 1, 0, 15], [0, 0, 1, floor_z], [0, 0, 0, 1]], float)

    boards = {}
    for key, (title, y, col) in MODELS.items():
        pl.add_mesh(pv.Cylinder(center=(0, y, floor_z + 0.2), direction=(0, 0, 1),
                                radius=4.2, height=0.4, resolution=64),
                    color="#2d313b", smooth_shading=True, specular=0.3)
        pl.add_mesh(pv.Disc(center=(0, y, floor_z + 0.42), inner=3.7, outer=4.2,
                            normal=(0, 0, 1), r_res=1, c_res=64),
                    color=col, ambient=0.5, diffuse=0.5)

        actors = []
        for mesh, c, smooth in board_parts(col) + frame_parts():
            actors.append(pl.add_mesh(mesh, color=c, smooth_shading=smooth,
                                      ambient=0.15, diffuse=0.7,
                                      specular=0.35, specular_power=25))
        set_pose(actors, (0, y, 0), 0, 0)
        boards[key] = (actors, (0, y, 0))

        pl.add_point_labels(np.array([[0, y + TITLE_SHIFT_PER_CHAR * len(title), 4.6]]), [title], font_size=20,
                            text_color=col, shape=None, show_points=False,
                            always_visible=True, bold=True)

    pl.add_text("left drag: rotate  |  wheel: zoom  |  middle drag: pan",
                position="lower_left", font_size=9, color="#9aa0ab")
    pl.camera_position = [(-28, -4, 11), (0, 0, -0.5), (0, 0, 1)]
    try:
        pl.enable_anti_aliasing("ssaa")
    except Exception:
        pass
    return boards


def apply_values(boards, v):
    acc_r, acc_p, ekf_r, ekf_p, comp_r, comp_p = v
    for key, (r, p) in (("raw", (acc_r, acc_p)),
                        ("comp", (comp_r, comp_p)),
                        ("ekf", (ekf_r, ekf_p))):
        actors, pos = boards[key]
        set_pose(actors, pos, r, p)


# ---------------------------------------------------------------- main
def main():
    src = DemoSource() if PORT.lower() == "demo" else SerialSource(PORT)

    pl = pv.Plotter(window_size=(1500, 850), title="IMU comparison")
    boards = build_scene(pl)

    def window_open():
        return not getattr(pl, "_closed", False) and getattr(pl, "ren_win", True) is not None

    try:
        pl.show(interactive_update=True, auto_close=False)   # opens the window, does not block
        while window_open():
            v = src.latest()
            if v is not None:
                apply_values(boards, v)
            try:
                pl.update(stime=UPDATE_MS, force_redraw=True)
            except Exception as exc:                         # window closed by the user
                if window_open():
                    print("render error:", exc)
                break
    finally:
        src.close()


if __name__ == "__main__":
    main()