import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import struct
from pathlib import Path

# =============================================
# Load data
# =============================================
df = pd.read_csv("full_state_output.csv")
t = df["t"]

def load_terrain(path):
    with open(path, "rb") as f:
        magic, rows, cols, x_min, x_max, y_min, y_max = struct.unpack("<8sII4d", f.read(48))
        z = np.fromfile(f, dtype=np.float64, count=rows * cols).reshape(rows, cols)
    x = np.linspace(x_min, x_max, cols)
    y = np.linspace(y_min, y_max, rows)
    return np.meshgrid(x, y), z
    
def load_trajectory(path):
    with open(path, "rb") as f:
        magic, count = struct.unpack("<8sI", f.read(12))
        data = np.fromfile(f, dtype=np.float64, count=count * 8).reshape(count, 8)
    return data[:, 1], data[:, 2], data[:, 3]

# =============================================
# Fig 1: 3D terrain + reference + actual path
# =============================================
(xg, yg), zg = load_terrain("terrain.bin")
ref_x, ref_y, ref_z = load_trajectory("terrain_traj.bin")

fig3d = plt.figure(figsize=(12, 9))
ax3d = fig3d.add_subplot(111, projection="3d")
stride = 4
ax3d.plot_surface(xg[::stride, ::stride], yg[::stride, ::stride], zg[::stride, ::stride],
                  cmap="gray", linewidth=0, antialiased=True, alpha=0.9)

lift = max(0.06 * float(np.ptp(zg)), 0.05)

# reference trajectory
ax3d.plot3D(ref_x, ref_y, ref_z + lift, color="white", linewidth=5, alpha=0.8)
ax3d.plot3D(ref_x, ref_y, ref_z + lift, color="red", linewidth=2.5, label="Reference")

# actual trajectory from CSV
act_z = np.interp(df["x"], np.linspace(xg.min(), xg.max(), zg.shape[1]),
                  zg[zg.shape[0]//2, :])  # rough z estimate
# better: use the terrain interp if available, or just use estimated z
ax3d.plot3D(df["x"], df["y"], df["z"] + lift, color="cyan", linewidth=2, label="Actual", zorder=10)
ax3d.plot3D(df["xe"], df["ye"], df["ze"] + lift, color="yellow", linewidth=1.5, linestyle="--", label="Estimated", zorder=20)
# start/end markers
ax3d.scatter(ref_x[0], ref_y[0], ref_z[0] + lift*2, color="green", edgecolors="white", s=120, zorder=10, label="Start")
ax3d.scatter(ref_x[-1], ref_y[-1], ref_z[-1] + lift*2, color="blue", edgecolors="white", s=120, zorder=10, label="End")

ax3d.set_xlabel("X (m)")
ax3d.set_ylabel("Y (m)")
ax3d.set_zlabel("Z (m)")
ax3d.set_title("Terrain + Reference + Actual + Estimated Trajectory")
ax3d.view_init(elev=45, azim=-35)
# after ax3d.view_init(...)
ax3d.set_box_aspect((np.ptp(xg), np.ptp(yg), max(np.ptp(zg), 1e-6)))
ax3d.legend()

# =============================================
# Fig 2: 9-DOF state true vs estimated
# =============================================
fig, axes = plt.subplots(3, 3, figsize=(16, 10), sharex=True)
fig.suptitle("Full 9-DOF State: True vs Estimated", fontsize=14)

pairs = [
    [("x","xe","X Pos (m)"), ("vx","vxe","X Vel (m/s)"), ("yaw","yaw_e","Yaw (rad)")],
    [("y","ye","Y Pos (m)"), ("vy","vye","Y Vel (m/s)"), ("pitch","pitch_e","Pitch (rad)")],
    [("z","ze","Z Pos (m)"), ("vz","vze","Z Vel (m/s)"), ("roll","roll_e","Roll (rad)")],
]

for row in range(3):
    for col in range(3):
        true_col, est_col, title = pairs[row][col]
        ax = axes[row, col]
        ax.plot(t, df[true_col], "b-", linewidth=1.5, label="True")
        ax.plot(t, df[est_col], "r--", linewidth=1.0, label="Est")
        ax.set_title(title)
        ax.grid(True, alpha=0.3)
        if row == 0 and col == 0:
            ax.legend()
        if row == 2:
            ax.set_xlabel("Time (s)")

plt.tight_layout()

# =============================================
# Fig 3: Controller + tracking error
# =============================================
fig2, axes2 = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
fig2.suptitle("Controller & Tracking", fontsize=14)

axes2[0].plot(t, df["wr"], "r", linewidth=1, label="wr")
axes2[0].plot(t, df["wl"], "b", linewidth=1, label="wl")
axes2[0].set_title("Wheel Speeds (rad/s)")
axes2[0].legend()
axes2[0].grid(True, alpha=0.3)

axes2[1].plot(t, df["s1"], "r", linewidth=1, label="s1 (x)")
axes2[1].plot(t, df["s2"], "b", linewidth=1, label="s2 (y)")
axes2[1].plot(t, df["s3"], "m", linewidth=1, label="s3 (yaw)")
axes2[1].axhline(0, color="k", linewidth=0.5)
axes2[1].set_title("Tracking Error")
axes2[1].legend()
axes2[1].grid(True, alpha=0.3)

axes2[2].plot(t, df["mu_r_est"], "r", linewidth=1, label="μ_R est")
axes2[2].plot(t, df["mu_l_est"], "b", linewidth=1, label="μ_L est")

axes2[2].plot(t, df["mu_r_act"], "g", linewidth=1, label="μ_R act")
axes2[2].plot(t, df["mu_l_act"], "m", linewidth=1, label="μ_L act")

axes2[2].axhline(1, color="k", linewidth=0.5, linestyle="--", label="No slip")
axes2[2].set_title("Estimated Slip Ratio")
axes2[2].set_xlabel("Time (s)")
axes2[2].legend()
axes2[2].grid(True, alpha=0.3)

plt.tight_layout()
plt.show()