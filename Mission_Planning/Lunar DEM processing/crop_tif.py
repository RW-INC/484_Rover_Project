import rasterio
from rasterio.windows import Window

# Replace with your actual file path
input_path = 'LDEM_83S_10MPP_ADJ.tiff'
output_path = 'LDEM_cropped.tiff'

with rasterio.open(input_path) as src:
    # Define the crop window (column_offset, row_offset, width, height)
    # For example, to crop a 500x500 square from the top-left (100, 100):
    win = Window(100, 100, 5000, 5000)

    # Read the data from that window
    cropped_data = src.read(window=win)

    # Copy and update the metadata (crs, transform, etc.) to match the crop
    kwargs = src.meta.copy()
    kwargs.update({
        'height': win.height,
        'width': win.width,
        'transform': src.window_transform(win)
    })

    with rasterio.open(output_path, 'w', **kwargs) as dst:
        dst.write(cropped_data)
