import cv2
import numpy as np
import math
import os

# Load original map
img = cv2.imread("finalmap.pgm", cv2.IMREAD_UNCHANGED)
# cv2.imshow('original_map.pgm', img)
# cv2.waitKey(0)
# Parameters from YAML
resolution = 0.05  # meters per pixel
origin = [-10.0, -10.0, 0.10472]  # last value is yaw in radians

# 1. Rotate the image
angle_deg = math.degrees(origin[2])  # convert yaw to degrees
(h, w) = img.shape
center = (w // 2, h // 2)
rotation_matrix = cv2.getRotationMatrix2D(center, angle_deg, 1.0)
rotated = cv2.warpAffine(img, rotation_matrix, (w, h), flags=cv2.INTER_NEAREST)

# 2. Save new map
cv2.imshow('rotated_map.pgm', rotated)
cv2.waitKey(0)
cv2.imwrite('rotated_map.pgm', rotated)

# 3. Update YAML manually or generate it
with open('rotated_map.yaml', 'w') as f:
    f.write(f"""image: rotated_map.pgm
resolution: {resolution}
origin: [0.0, 0.0, 0.0]  # now zero rotation
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
""")