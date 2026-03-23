from pdf2image import convert_from_path
import cv2
import numpy as np

PDF_PATH = "south_pole_slope_map.pdf"
DPI = 300 

# Convert PDF to image
pages = convert_from_path(PDF_PATH, dpi=DPI)
img = np.array(pages[0])  # take first page
img = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)

red_lower1 = np.array([0, 100, 100])
red_upper1 = np.array([10, 255, 255])
red_lower2 = np.array([160, 100, 100])
red_upper2 = np.array([180, 255, 255])

green_lower = np.array([35, 80, 80])
green_upper = np.array([85, 255, 255])

yellow_lower = np.array([20, 80, 80])
yellow_upper = np.array([35, 255, 255])

red_mask1 = cv2.inRange(img, red_lower1, red_upper1)
red_mask2 = cv2.inRange(img, red_lower2, red_upper2)
red_mask = red_mask1 + red_mask2

green_mask = cv2.inRange(img, green_lower, green_upper)
yellow_mask = cv2.inRange(img, yellow_lower, yellow_upper)

# count pixels
total_pixels = img.shape[0] * img.shape[1]
red_pixels = np.count_nonzero(red_mask)
green_pixels = np.count_nonzero(green_mask)
yellow_pixels = np.count_nonzero(yellow_mask)

# compute percentages
red_pct = 100 * red_pixels / total_pixels
green_pct = 100 * green_pixels / total_pixels
yellow_pct = 100 * yellow_pixels / total_pixels

print(f"Red: {red_pct:.2f}%")
print(f"Green: {green_pct:.2f}%")
print(f"Yellow: {yellow_pct:.2f}%")
print(f"Other: {100 - (red_pct + green_pct + yellow_pct):.2f}%")
