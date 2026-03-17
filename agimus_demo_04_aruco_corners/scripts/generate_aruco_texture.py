#!/usr/bin/env python3
"""
Generate an ArUco marker PNG texture for the Gazebo simulation model.

Usage:
    python3 generate_aruco_texture.py

Generates:
    ../models/aruco_marker_0/materials/textures/aruco_4x4_id0.png

The marker parameters match those used by aruco_node in bringup.launch.py:
  - Dictionary: DICT_4X4_50
  - Marker ID:  0
"""

import os
import sys

try:
    import cv2
except ImportError:
    print("ERROR: opencv-python is required.")
    print("  pip install opencv-python")
    sys.exit(1)


DICT_ID = cv2.aruco.DICT_4X4_50
MARKER_ID = 0
IMAGE_SIZE = 512  # pixels - large enough for clean rendering


def main():
    aruco_dict = cv2.aruco.getPredefinedDictionary(DICT_ID)
    # generateImageMarker added in OpenCV 4.7; use drawMarker for older versions (Ubuntu Noble = 4.6)
    if hasattr(cv2.aruco, "generateImageMarker"):
        marker_img = cv2.aruco.generateImageMarker(aruco_dict, MARKER_ID, IMAGE_SIZE, borderBits=1)
    else:
        marker_img = cv2.aruco.drawMarker(aruco_dict, MARKER_ID, IMAGE_SIZE)

    script_dir = os.path.dirname(os.path.abspath(__file__))
    output_dir = os.path.join(
        script_dir, "..", "models", "aruco_marker_0", "materials", "textures"
    )
    os.makedirs(output_dir, exist_ok=True)

    output_path = os.path.join(output_dir, "aruco_4x4_id0.png")
    cv2.imwrite(output_path, marker_img)
    print(f"Generated: {os.path.realpath(output_path)}")


if __name__ == "__main__":
    main()
