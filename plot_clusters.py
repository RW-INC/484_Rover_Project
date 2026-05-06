import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse

point_file = "cloud_0000.csv"
cluster_file = "build/clusters.csv"

points = pd.read_csv(point_file)
clusters = pd.read_csv(cluster_file)

fig, ax = plt.subplots(figsize=(8, 10))

ax.scatter(points["x"], points["y"], s=1, alpha=0.5)

k = 2.0  # 2-sigma ellipse

for _, c in clusters.iterrows():
    cx = c["center_x"]
    cy = c["center_y"]

    major_var = c["principal_variance"]
    minor_var = c["secondary_variance"]

    ux = c["principal_axis_x"]
    uy = c["principal_axis_y"]

    principal_axis = np.array([ux, uy])
    principal_axis = principal_axis / np.linalg.norm(principal_axis)

    secondary_axis = np.array([-principal_axis[1], principal_axis[0]])

    major_radius = k * np.sqrt(major_var)
    minor_radius = k * np.sqrt(minor_var)

    angle = np.degrees(np.arctan2(principal_axis[1], principal_axis[0]))

    ellipse = Ellipse(
        xy=(cx, cy),
        width=2 * major_radius,
        height=2 * minor_radius,
        angle=angle,
        fill=False,
        linewidth=2
    )

    ax.add_patch(ellipse)

    ax.scatter(cx, cy, s=25)

    ax.plot(
        [cx - major_radius * principal_axis[0], cx + major_radius * principal_axis[0]],
        [cy - major_radius * principal_axis[1], cy + major_radius * principal_axis[1]],
        linewidth=2
    )

    ax.plot(
        [cx - minor_radius * secondary_axis[0], cx + minor_radius * secondary_axis[0]],
        [cy - minor_radius * secondary_axis[1], cy + minor_radius * secondary_axis[1]],
        linewidth=2,
        linestyle="--"
    )

ax.set_aspect("equal", adjustable="box")
ax.set_xlabel("x")
ax.set_ylabel("y")
ax.set_title("Top-Down Clusters with PCA Ellipses and Axes")
plt.show()