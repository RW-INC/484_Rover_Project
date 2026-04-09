import rasterio # library useful for .tiff file handling
from rasterio.windows import from_bounds, Window # for choosing window size
import numpy as np
import cv2
import csv

def save_elevation_to_csv(input_tiff_filepath, region_size):
    with rasterio.open(input_tiff_filepath) as src:
        # elevation data
        window = from_bounds(-region_size, -region_size, region_size, region_size, src.transform)
        elevation = src.read(1, window = window)
        rows, cols = elevation.shape

        output_file = "elevation_data_040526.csv"
        transform = src.window_transform(window)
        transform_affine = src.window_transform(window)

        # for lat lon
        crs = src.crs
        geo_crs = src.crs.geodetic_crs

        row, col = 0, 0
        x, y = transform * (col, row)
        print('===== Here is some coordinate information: =====')
        print(f'Pixel coordinates: {col, row}')
        print(f'Projected stereographic coordinates: {x, y}')

        lon, lat = transform(crs, geo_crs, [x], [y])

        with open(output_file, 'w') as f:
            f.write("x,y,lon,lat,elevation\n")

            for i in range(rows):
                for j in range(cols):
                    x, y = transform_affine * (j, i)

                    lon, lat = transform(crs, geo_crs, [x], [y])

                    f.write(f"{j},{i},{lon[0]},{lat[0]},{elevation[i,j]}\n")

        # with open(output_file, mode='w', newline='') as f:
        #     writer = csv.writer(f)
            
        #     # header
        #     writer.writerow(["row", "col", "elevation"])
            
        #     for i in range(rows):
        #         for j in range(cols):
        #             writer.writerow([i, j, elevation[i, j]])

def convert_path_pixel_to_meters(filepath, write_file = False):
    with open(filepath, mode='r') as file:
        reader = csv.reader(file)
        full_path_x = []
        full_path_y = []
        
        # skip first two lines
        next(reader)
        next(reader)
        
        # read each waypoint
        for row in reader:
            full_path_y.append(float(row[1]))
            full_path_x.append(float(row[0]))

    full_path_pixels = np.array([full_path_x, full_path_y]).T
    full_path_meters = full_path_pixels * 10

    csv_file_path = filepath[:-4] + "_METERS.csv"

    if write_file:
        with open(csv_file_path, mode='w', newline='') as file:
            writer = csv.writer(file)
            
            # write header
            writer.writerow(["x", "y"])
            
            # write each part of path
            for node in full_path_meters:
                writer.writerow([node[0], node[1]])
    
        print(f"The path has been stored successfully inside '{csv_file_path}'.")

    return full_path_meters

def convert_path_meter_to_pixels(filepath, write_file = False):
    with open(filepath, mode='r') as file:
        reader = csv.reader(file)
        full_path_x = []
        full_path_y = []
        
        # skip first two lines
        next(reader)
        next(reader)
        
        # read each waypoint
        for row in reader:
            full_path_y.append(float(row[1]))
            full_path_x.append(float(row[0]))

    full_path_m = np.array([full_path_x, full_path_y]).T

    full_path_pixels = full_path_m / 10

    csv_file_path = filepath[:-4] + "_PIXELS.csv"

    if write_file:
        with open(csv_file_path, mode='w', newline='') as file:
            writer = csv.writer(file)
            
            # write header
            writer.writerow(["x", "y"])
            
            # write each part of path
            for node in full_path_pixels:
                writer.writerow([node[0], node[1]])
    
        print(f"The path has been stored successfully inside '{csv_file_path}'.")

    return full_path_pixels

def generate_Dstar_tiff(full_path, tiff_file, region_size, buffer_pixels, output_file):
    # open the original tiff and grab data based on window that full path is based on
    with rasterio.open(tiff_file) as src:
        # elevation data
        window = from_bounds(-region_size, -region_size, region_size, region_size, src.transform)
        elevation = src.read(1, window = window)
        h, w = elevation.shape

        scale = 10 # increasing resolution by 10x for 1 m / pixel

        height = h*scale
        width = w*scale

        # resize elevation data to be 1 m / pixel
        elevation_1mpp = cv2.resize(elevation, (width, height), interpolation = cv2.INTER_CUBIC)

        transform = src.window_transform(window)
        transform = transform*rasterio.Affine.scale(0.1,0.1)

    scale = 10

    x = full_path[:, 0]*scale
    y = full_path[:, 1]*scale

    # determine x y limits with buffer
    # x_min = int(max(x.min() - buffer_pixels, 0))
    # x_max = int(min(x.max() + buffer_pixels, width))
    # y_min = int(max(y.min() - buffer_pixels, 0))
    # y_max = int(min(y.max() + buffer_pixels, height))

    x_min = int(np.floor(max(x.min() - buffer_pixels, 0)))
    x_max = int(np.ceil(min(x.max() + buffer_pixels, width)))
    y_min = int(np.floor(max(y.min() - buffer_pixels, 0)))
    y_max = int(np.ceil(min(y.max() + buffer_pixels, height)))

    # crop data
    cropped_array = elevation_1mpp[y_min:y_max+1, x_min:x_max+1]
    cropped_array = cropped_array[np.newaxis, :, :]

    # save to new TIFF
    with rasterio.open(
        output_file, 'w', driver='GTiff',
        height=cropped_array.shape[1], width=cropped_array.shape[2],
        count=1,
        dtype=cropped_array.dtype,
        crs=src.crs, transform=transform*rasterio.Affine.translation(x_min, y_min),) as dst:
        dst.write(cropped_array)
    print(f'Saved DEM cropped file {output_file} with following bounds:')
    print(f'x min: {x_min}')
    print(f'x max: {x_max}')
    print(f'y min: {y_min}')
    print(f'y max: {y_max}')

    print("DEM size (1mpp):", width, height)
    print("Path X range:", x.min(), x.max())
    print("Path Y range:", y.min(), y.max())

    x_len = x_max - x_min + 1
    y_len = y_max - y_min + 1

    area = x_len*y_len

    scale = 1 # in meters / pixel

    print(f'Size of file (pixels): {x_len} by {y_len} pixels')
    print(f'Size of file (meters): {x_len*scale} by {y_len*scale} meters')
    print(f'Area of .tiff file: {int(area*(scale**2))} square meters')

    return x_min, y_min

def output_cropped_path_coords(x_min, y_min, full_path_pixels, output_csv):
    new_path = []
    scale = 10

    for node in full_path_pixels:
        x_old, y_old = node*scale
        x_new = x_old - x_min
        y_new = y_old - y_min
        new_path.append([x_new, y_new])

    new_path = np.array(new_path)

    # save to CSV
    with open(output_csv, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["x", "y"])
        for node in new_path:
            writer.writerow([node[0], node[1]])

    print(f"Saved cropped path coordinates (in pixels) to {output_csv}")

region_size = 15000
buffer_pixels = 100

input_tiff_filepath = "data/LDEM_83S_10MPP_ADJ.tiff"

input_csv_filepath = "planned_path_nodes_meters_30.0.csv"

output_tiff_filepath = "LDEM_30m_path_1mpp.tiff"

output_csv_filepath = "30m_path_PIXELS.csv"

full_path_pixels = convert_path_meter_to_pixels(input_csv_filepath, write_file = False)

x_min, y_min = generate_Dstar_tiff(full_path_pixels, input_tiff_filepath, region_size, buffer_pixels, output_tiff_filepath)

output_cropped_path_coords(x_min, y_min, full_path_pixels, output_csv_filepath)

