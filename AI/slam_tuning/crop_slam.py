# Re-import necessary libraries after code execution environment reset
import cv2
import yaml
import math
import numpy as np

# Re-define paths for the previously uploaded files
pgm_path = "rotated_map.pgm"
yaml_path = "rotated_map.yaml"

# Load the YAML file
with open(yaml_path, 'r') as f:
    map_metadata = yaml.safe_load(f)

# Extract relevant information
resolution = map_metadata.get('resolution', 0.05)
origin = map_metadata.get('origin', [0.0, 0.0, 0.0])  # x, y, yaw
yaw_rad = origin[2]
yaw_deg = math.degrees(yaw_rad)

# Load the PGM image using OpenCV
img = cv2.imread(pgm_path, cv2.IMREAD_UNCHANGED)
(h, w) = img.shape

# Rotate the image around its center using the yaw angle
center = (w // 2, h // 2)
rotation_matrix = cv2.getRotationMatrix2D(center, yaw_deg, 1.0)
rotated_img = cv2.warpAffine(img, rotation_matrix, (w, h), flags=cv2.INTER_NEAREST)

# Coordinates to crop (in meters)
x_min_m, x_max_m = -6, 2
y_min_m, y_max_m = -4, 4

# Convert coordinates to pixel indices
center_px_x = rotated_img.shape[1] // 2
center_px_y = rotated_img.shape[0] // 2

x_min_px = int(center_px_x + x_min_m / resolution)
x_max_px = int(center_px_x + x_max_m / resolution)
y_max_px = int(center_px_y - y_min_m / resolution)
y_min_px = int(center_px_y - y_max_m / resolution)

# Ensure indices are within bounds
x_min_px = max(0, x_min_px)
x_max_px = min(rotated_img.shape[1], x_max_px)
y_min_px = max(0, y_min_px)
y_max_px = min(rotated_img.shape[0], y_max_px)

# Crop the image
custom_crop_img = rotated_img[y_min_px:y_max_px, x_min_px:x_max_px]
cv2.imshow("crop_img", custom_crop_img)
cv2.waitKey(0)
# Save cropped image
custom_cropped_pgm_path = "final_map.pgm"
cv2.imwrite(custom_cropped_pgm_path, custom_crop_img)

# Generate YAML for cropped map
custom_crop_yaml = {
    'image': 'final_map.pgm',
    'resolution': resolution,
    'origin': [x_min_m, y_min_m, 0.0],
    'negate': map_metadata.get('negate', 0),
    'occupied_thresh': map_metadata.get('occupied_thresh', 0.65),
    'free_thresh': map_metadata.get('free_thresh', 0.196)
}

custom_crop_yaml_path = "final_map.yaml"
with open(custom_crop_yaml_path, 'w') as f:
    yaml.dump(custom_crop_yaml, f)

(custom_cropped_pgm_path, custom_crop_yaml_path)
