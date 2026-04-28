from __future__ import annotations

import argparse
import struct
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Plot terrain.bin with an optional terrain_traj.bin overlay.")
    parser.add_argument("terrain_path", nargs="?", default="terrain.bin")
    # parser.add_argument("control_path", nargs="?", default="simulation_output.bin")
    parser.add_argument("terrain_traj_path", nargs="?", default="terrain_traj.bin")

    parser.add_argument("--trajectory", default=None)
    parser.add_argument("--surface-stride", type=int, default=4)
    return parser.parse_args()


def resolve_path(path_str: str) -> Path:
    requested = Path(path_str)
    fallback_requested = requested

    if requested.is_absolute():
        if requested.exists():
            return requested
        if fallback_requested != requested and fallback_requested.exists():
            return fallback_requested

    root = Path(__file__).resolve().parent.parent
    for candidate in (
        Path.cwd() / requested,
        Path.cwd() / fallback_requested,
        Path(__file__).resolve().parent / requested,
        Path(__file__).resolve().parent / fallback_requested,
        root / requested,
        root / fallback_requested,
        root / "build" / requested,
        root / "build" / fallback_requested,
        root / "build" / "Debug" / requested,
        root / "build" / "Debug" / fallback_requested,
        root / "build" / "Release" / requested,
        root / "build" / "Release" / fallback_requested,
    ):
        if candidate.exists():
            return candidate

    raise FileNotFoundError(f"Could not find '{path_str}'.")


def load_terrain(path: Path) -> dict[str, np.ndarray]:
    with path.open("rb") as handle:
        magic, rows, cols, x_min, x_max, y_min, y_max = struct.unpack("<8sII4d", handle.read(48))
        if magic.rstrip(b"\0") != b"SPXTER1":
            raise ValueError(f"Unrecognized terrain format in {path}.")
        z_grid = np.fromfile(handle, dtype=np.float64, count=rows * cols).reshape(rows, cols)

    x_axis = np.linspace(x_min, x_max, cols)
    y_axis = np.linspace(y_min, y_max, rows)
    x_grid, y_grid = np.meshgrid(x_axis, y_axis)
    return {"x_grid": x_grid, "y_grid": y_grid, "z_grid": z_grid}


def load_trajectory(path: Path) -> dict[str, np.ndarray]:
    with path.open("rb") as handle:
        magic, count = struct.unpack("<8sI", handle.read(12))
        if magic.rstrip(b"\0") != b"SPXTRJ1":
            raise ValueError(f"Unrecognized trajectory format in {path}.")
        data = np.fromfile(handle, dtype=np.float64, count=count * 8).reshape(count, 8)

    return {
        "t": data[:, 0],
        "x": data[:, 1],
        "y": data[:, 2],
        "z": data[:, 3],
    }


def find_trajectory_file(arg_value: str | None, terrain_path: Path) -> Path | None:
    if arg_value:
        return resolve_path(arg_value)

    candidate = terrain_path.with_name("terrain_traj.bin")
    return candidate if candidate.exists() else None


def main() -> None:
    args = parse_args()
    terrain_path = resolve_path(args.terrain_path)
    # control_path = resolve_path(args.control_path)

    traj_path = find_trajectory_file(args.trajectory, terrain_path)
    terrain = load_terrain(terrain_path)


    z_grid = terrain["z_grid"]
    x_grid = terrain["x_grid"]
    y_grid = terrain["y_grid"]

    fig = plt.figure(figsize=(11, 8))
    ax = fig.add_subplot(111, projection="3d")
    surface = ax.plot_surface(
        x_grid[:: args.surface_stride, :: args.surface_stride],
        y_grid[:: args.surface_stride, :: args.surface_stride],
        z_grid[:: args.surface_stride, :: args.surface_stride],
        cmap="gray",
        linewidth=0,
        antialiased=True,
        alpha=1.0,
    )

    ax.set_box_aspect((np.ptp(x_grid), np.ptp(y_grid), max(np.ptp(z_grid), 1e-6)))
    ax.set_title(f"Terrain Surface\n{terrain_path.name}")
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_zlabel("z")
    ax.view_init(elev=45, azim=-35)

    if traj_path is not None:
        traj = load_trajectory(traj_path)
        lift = max(0.06 * float(np.ptp(z_grid)), 0.05)
        traj_z = traj["z"] + lift

        ax.plot3D(traj["x"], traj["y"], traj_z, color="white", linewidth=6.0, alpha=0.85)
        ax.plot3D(traj["x"], traj["y"], traj_z, color="red", linewidth=3.2, zorder=10)
        ax.scatter(traj["x"][0], traj["y"][0], traj_z[0] * 1.5, color="tab:green", edgecolors="white", linewidths=1.5, s=110, depthshade=False)
        ax.scatter(traj["x"][-1], traj["y"][-1], traj_z[-1] * 1.5, color="tab:blue", edgecolors="white", linewidths=1.5, s=110, depthshade=False)
        ax.set_title(f"Terrain Surface with Trajectory\n{terrain_path.name}")

    # if control_path is not None:
    #     with control_path.open("rb") as handle:
    #         magic, count = struct.unpack("<8sI", handle.read(12))
    #         if magic.rstrip(b"\0") != b"SPXSIM1":
    #             raise ValueError(f"Unrecognized simulation output format in {control_path}.")
    #         data = np.fromfile(handle, dtype=np.float64, count=count * 9).reshape(count, 9)

        # sim_traj = {
        #     "t": data[:, 0],
        #     "x": data[:, 1],
        #     "y": data[:, 2],
        #     "z": data[:, 3],
        # }
        # print(sim_traj['x'])
        # print(sim_traj['y'])
        # print(sim_traj['z'])
        
        # ax.plot3D(sim_traj["x"], sim_traj["y"], sim_traj["z"], color="cyan", linewidth=3.0, alpha=0.85, zorder=10)
        # ax.set_title(f"Terrain Surface with Trajectory and Simulation\n{terrain_path.name}")
    
    ax.legend(["Trajectory", "Start", "End"])
    fig.colorbar(surface, ax=ax, shrink=0.7, pad=0.08, label="height")
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
