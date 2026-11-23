
"""
Live MPU6050 Roll/Pitch comparison:
Accelerometer vs EKF vs Complementary Filter

Input: 9 CSV values from STM32 at 115200 baud.
Run: python imu_viewer.py
"""

import time
from collections import deque

import serial
import matplotlib.pyplot as plt
from matplotlib.ticker import MultipleLocator

# ==========================================
# CONFIGURATION
# ==========================================

PORT = "COM5"
BAUD = 115200

N = 300                 # 3 seconds at 100 Hz
HALF = 25               # Y-axis follows EKF +/- 25 degrees
DRAW_PERIOD = 0.05      # 20 FPS

CPU_HZ = 84_000_000

# ==========================================
# SERIAL CONNECTION
# ==========================================

ser = serial.Serial(PORT, BAUD, timeout=0.001)
ser.reset_input_buffer()

rxbuf = bytearray()

print(f"Connected to STM32: {PORT} @ {BAUD}")

# ==========================================
# DATA BUFFERS
# ==========================================

# 6 angle signals
buf = [deque([0.0] * N, maxlen=N) for _ in range(6)]

# CPU cycle history
cyc_acc = deque(maxlen=500)
cyc_ekf = deque(maxlen=500)
cyc_comp = deque(maxlen=500)

x = list(range(N))

# ==========================================
# FIGURE
# ==========================================

plt.ion()

fig, (ax_r, ax_p) = plt.subplots(
    2, 1,
    sharex=True,
    figsize=(13, 8)
)

fig.suptitle(
    "MPU6050 - Filter Performance Comparison",
    fontsize=15,
    fontweight="bold"
)

# ==========================================
# COLORS
# ==========================================

ACC_COLOR = "tab:orange"
EKF_COLOR = "tab:blue"
COMP_COLOR = "tab:green"

# ==========================================
# ROLL CURVES
# ==========================================

l_acc_r, = ax_r.plot(
    x, buf[0],
    color=ACC_COLOR,
    linewidth=1.3,
    alpha=0.85,
    label="Raw Accelerometer"
)

l_ekf_r, = ax_r.plot(
    x, buf[2],
    color=EKF_COLOR,
    linewidth=2,
    label="Extended Kalman Filter"
)

l_comp_r, = ax_r.plot(
    x, buf[4],
    color=COMP_COLOR,
    linewidth=2,
    label="Complementary Filter"
)

# ==========================================
# PITCH CURVES
# ==========================================

l_acc_p, = ax_p.plot(
    x, buf[1],
    color=ACC_COLOR,
    linewidth=1.3,
    alpha=0.85,
    label="Raw Accelerometer"
)

l_ekf_p, = ax_p.plot(
    x, buf[3],
    color=EKF_COLOR,
    linewidth=2,
    label="Extended Kalman Filter"
)

l_comp_p, = ax_p.plot(
    x, buf[5],
    color=COMP_COLOR,
    linewidth=2,
    label="Complementary Filter"
)

# ==========================================
# AXIS CONFIGURATION
# ==========================================

for ax in (ax_r, ax_p):

    ax.set_ylim(-HALF, HALF)

    # One major division = 2 degrees
    ax.yaxis.set_major_locator(MultipleLocator(5))
    ax.tick_params(axis="y", pad=15, labelsize=10)
    # Only horizontal major grid lines
    ax.grid(
        True,
        axis="y",
        which="major",
        linestyle="--",
        alpha=0.35
    )

    ax.minorticks_off()
    ax.set_ylabel("Angle (degrees)")
    ax.legend(loc="upper right")

ax_r.set_title("ROLL")
ax_p.set_title("PITCH")
ax_p.set_xlabel("Samples (100 Hz)")

# ==========================================
# CPU CYCLES DISPLAY
# ==========================================

cpu_display = fig.text(
    0.5, 0.015,
    "",
    ha="center",
    fontsize=10,
    family="monospace"
)

plt.tight_layout(rect=[0, 0.07, 1, 0.95])
# ==========================================
# CPU STATISTICS
# ==========================================

def cpu_average(data):

    if not data:
        return "Waiting..."

    avg = sum(data) / len(data)
    us = avg / CPU_HZ * 1_000_000

    return f"{avg:,.0f} cycles ({us:.2f} us)"


# ==========================================
# SERIAL DATA RECEPTION
# ==========================================

last_values = None

def read_serial():

    global rxbuf, last_values

    # Read all currently available bytes
    rxbuf.extend(ser.read(ser.in_waiting or 1))

    while b"\n" in rxbuf:

        raw, _, remaining = rxbuf.partition(b"\n")
        rxbuf = bytearray(remaining)

        try:

            parts = raw.decode(
                errors="ignore"
            ).strip().split(",")

            if len(parts) != 9:
                continue

            v = [float(p) for p in parts]

            # Angles
            buf[0].append(v[0])  # Acc Roll
            buf[1].append(v[1])  # Acc Pitch

            buf[2].append(v[2])  # EKF Roll
            buf[3].append(v[3])  # EKF Pitch

            buf[4].append(v[4])  # Comp Roll
            buf[5].append(v[5])  # Comp Pitch

            # CPU cycles
            cyc_acc.append(v[6])
            cyc_ekf.append(v[7])
            cyc_comp.append(v[8])

            last_values = v

        except (ValueError, OverflowError):
            continue


# ==========================================
# MAIN PLOTTING LOOP
# ==========================================

last_draw = 0.0

try:

    plt.show(block=False)

    while plt.fignum_exists(fig.number):

        # Read incoming STM32 data
        read_serial()

        now = time.perf_counter()

        if last_values is not None and now - last_draw >= DRAW_PERIOD:

            last_draw = now

            # Update curve data
            l_acc_r.set_ydata(buf[0])
            l_ekf_r.set_ydata(buf[2])
            l_comp_r.set_ydata(buf[4])

            l_acc_p.set_ydata(buf[1])
            l_ekf_p.set_ydata(buf[3])
            l_comp_p.set_ydata(buf[5])

            # Dynamic Y-axis centered on EKF
            current_ekf_roll = last_values[2]
            current_ekf_pitch = last_values[3]

            ax_r.set_ylim(
                current_ekf_roll - HALF,
                current_ekf_roll + HALF
            )

            ax_p.set_ylim(
                current_ekf_pitch - HALF,
                current_ekf_pitch + HALF
            )

            # Live CPU cycle averages
            cpu_display.set_text(
                f"CPU AVERAGE  |  "
                f"ACC: {cpu_average(cyc_acc)}  |  "
                f"EKF: {cpu_average(cyc_ekf)}  |  "
                f"COMP: {cpu_average(cyc_comp)}"
            )

            # Refresh window
            fig.canvas.draw_idle()
            fig.canvas.flush_events()

        time.sleep(0.001)

finally:

    ser.close()

    print("\n========== FINAL CPU RESULTS ==========")

    for name, data in [
        ("Accelerometer", cyc_acc),
        ("EKF", cyc_ekf),
        ("Complementary", cyc_comp)
    ]:

        if data:

            avg = sum(data) / len(data)
            minimum = min(data)
            maximum = max(data)

            print(f"\n{name}:")
            print(f"  Average: {avg:.0f} cycles")
            print(f"  Minimum: {minimum:.0f} cycles")
            print(f"  Maximum: {maximum:.0f} cycles")
            print(f"  Average time: {avg / CPU_HZ * 1e6:.2f} us")

    print("=======================================")
