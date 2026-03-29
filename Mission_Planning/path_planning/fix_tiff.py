import rasterio
import numpy as np

def fix_lunar_tiff(input_path, output_path):
    with rasterio.open(input_path) as src:
        # Read the first band of the TIFF
        data = src.read(1)
        
        # Identify the NoData value from metadata or find the extreme minimum
        nodata_val = src.nodata if src.nodata is not None else np.min(data)
        
        # Create a mask for "spikes": 
        # Typically, values <= -10000 are error placeholders in NASA DEMs
        spike_mask = (data <= -10000) | (data == nodata_val)
        
        # Calculate a neutral replacement value (the median of real terrain)
        valid_terrain = data[~spike_mask]
        replacement = np.median(valid_terrain) if valid_terrain.size > 0 else 0
        
        # Replace spikes with the neutral value
        fixed_data = np.where(spike_mask, replacement, data)
        
        # Save the fixed file with original geospatial metadata
        kwargs = src.meta.copy()
        kwargs.update(dtype=rasterio.float32, nodata=replacement)
        
        with rasterio.open(output_path, 'w', **kwargs) as dst:
            dst.write(fixed_data.astype(rasterio.float32), 1)

# Run the fix
fix_lunar_tiff('LDEM_cropped.tiff', 'LDEM_fixed.tiff')
