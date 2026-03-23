import rasterio
from rasterio.windows import from_bounds
import numpy as np
import matplotlib.pyplot as plt
from scipy.io import savemat

# =====================================
# PARAMETERS
# =====================================
region_length = 25000   # meters (50 km x 50 km window)
dem_file = "LDEM_83S_10MPP_ADJ.tiff"
output_mat_file = "south_pole_elevation.mat"


# =====================================
# LOAD DEM + BUILD GRID
# =====================================
def generate_elevation_grid(region_length):

    with rasterio.open(dem_file) as src:

        window = from_bounds(
            -region_length,
            -region_length,
            region_length,
            region_length,
            src.transform
        )

        elevations = src.read(1, window=window)
        transform = src.window_transform(window)

    height, width = elevations.shape

    # Create coordinate vectors
    x = np.linspace(-region_length, region_length, width)
    y = np.linspace(-region_length, region_length, height)

    X, Y = np.meshgrid(x, y)
    return X, Y, elevations

def downsample(X, Y, elevations, factor):
    return (
        X[::factor, ::factor],
        Y[::factor, ::factor],
        elevations[::factor, ::factor]
    )


# =====================================
# SAVE FOR MATLAB
# =====================================
def save_for_matlab(X, Y, elevations):

    # Save .mat file (best option)
    savemat(output_mat_file, {
        "X": X / 1000.0,
        "Y": Y / 1000.0,
        "elevation": elevations / 1000.0
    })

    print("Saved MATLAB file:", output_mat_file)


# =====================================
# PLOT ELEVATION MAP
# =====================================
def plot_elevation(X, Y, elevations):

    fig, ax = plt.subplots(figsize=(8, 8))

    extent = [
        X.min()/1000, X.max()/1000,
        Y.min()/1000, Y.max()/1000
    ]

    im = ax.imshow(
        elevations,
        extent=extent,
        origin='upper',
        cmap='terrain'
    )

    ax.set_title("Lunar South Pole Elevation Map", fontsize=16)
    ax.set_xlabel("Kilometers East")
    ax.set_ylabel("Kilometers North")
    ax.set_aspect('equal')

    cbar = plt.colorbar(im, ax=ax)
    cbar.set_label("Elevation (meters)")

    plt.tight_layout()
    plt.savefig("south_pole_elevation_map.png", dpi=300)
    plt.show()


# =====================================
# MAIN
# =====================================
if __name__ == "__main__":

    X, Y, elevations = generate_elevation_grid(region_length)

    # Downsample factor
    factor = 100   # 10 m × 100 = 1000 m resolution

    X, Y, elevations = downsample(X, Y, elevations, factor)

    save_for_matlab(X, Y, elevations)
    plot_elevation(X, Y, elevations)
