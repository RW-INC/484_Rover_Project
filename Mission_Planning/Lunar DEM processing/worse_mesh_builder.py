import numpy as np
from PIL import Image

# Load your 512 image
img = Image.open('heightmap_512smooth.png').convert('L')
# Resize down slightly for physics stability (256x256 is plenty for a 150m area)
img = img.resize((256, 256), Image.LANCZOS)
data = np.array(img)

width, height, max_z = 10.0, 10.0, 1.0
rows, cols = data.shape

with open('lunar_terrain.obj', 'w') as f:
    for r in range(rows):
        for c in range(cols):
            x = (c / (cols - 1)) * width - (width / 2)
            y = (r / (rows - 1)) * height - (height / 2)
            z = (data[r, c] / 255.0) * max_z
            f.write(f"v {x:.1f} {y:.1f} {z:.1f}\n")
    
    for r in range(rows - 1):
        for c in range(cols - 1):
            p1 = r * cols + c + 1
            p2 = r * cols + (c + 1) + 1
            p3 = (r + 1) * cols + c + 1
            p4 = (r + 1) * cols + (c + 1) + 1
            # Using standard triangle faces
            f.write(f"f {p1} {p2} {p4}\n")
            f.write(f"f {p1} {p4} {p3}\n")