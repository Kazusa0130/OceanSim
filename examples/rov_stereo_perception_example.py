#!/usr/bin/env python3
"""
ROV Stereo Camera Perception Example

This example demonstrates how to use the stereo camera as the main perception sensor
for ROV (Remotely Operated Vehicle) navigation and inspection tasks.

The stereo camera is mounted on the ROV and provides:
- Left and right camera images for stereo vision
- Depth estimation capabilities for obstacle avoidance
- Configurable viewing modes (forward, downward, upward)
"""

import numpy as np
from isaacsim.oceansim.sensors.StereoUWCamera import StereoUWCamera


def main():
    """Example of using stereo camera with ROV."""

    print("=" * 60)
    print("ROV Stereo Camera Perception Example")
    print("=" * 60)

    # Example 1: Create stereo camera mounted on ROV
    print("\n[1] Creating stereo camera for ROV...")

    # The camera is mounted at the front of the ROV
    # prim_path_prefix ensures it's a child of the ROV prim
    stereo_cam = StereoUWCamera(
        prim_path_prefix="/World/ROV/stereo_camera",
        name="ROV_Perception_Camera",
        resolution=[1920, 1080],
        baseline=0.12,  # 12cm baseline suitable for medium-range ROV operations
        focal_length=2.1,
        translation=np.array([0.3, 0.0, 0.1]),  # Front of ROV, slightly up
        yaml_config_path="/root/isaacsim/extsUser/OceanSim/config/stereo_camera.yaml"
    )

    print(f"   Camera baseline: {stereo_cam.get_baseline()}m")
    print(f"   Camera resolution: {stereo_cam.get_resolution()}")
    print(f"   Camera intrinsics: {stereo_cam.get_intrinsics()}")

    # Example 2: Set perception mode for different ROV tasks
    print("\n[2] Setting ROV perception modes...")

    # Mode 1: Forward-looking for navigation
    forward_orientation = stereo_cam.set_rov_perception_mode('forward')
    print("   Mode: Forward-looking for navigation")

    # Mode 2: Downward-looking for seabed inspection
    downward_orientation = stereo_cam.set_rov_perception_mode('downward')
    print("   Mode: Downward-looking for seabed inspection")

    # Mode 3: Angled for terrain following
    angled_orientation = stereo_cam.set_rov_perception_mode('angled')
    print("   Mode: Angled for terrain following")

    # Example 3: Depth estimation for obstacle avoidance
    print("\n[3] Depth estimation capabilities...")

    # Get depth range from configuration
    min_depth, max_depth = stereo_cam.get_rov_depth_range()
    print(f"   Measurable depth range: {min_depth}m - {max_depth}m")

    # Calculate depth from disparity examples
    disparities = [50, 100, 200]  # Example disparity values in pixels
    print("   Example depth calculations:")
    for disp in disparities:
        depth = stereo_cam.get_depth_from_disparity(disp)
        print(f"      Disparity {disp}px -> Depth {depth:.2f}m")

    # Example 4: Initialize camera for operation
    print("\n[4] Initializing stereo camera...")
    print("   (In actual use, call stereo_cam.initialize() with appropriate parameters)")

    # stereo_cam.initialize(
    #     UW_param=np.array([0.0, 0.31, 0.24, 0.05, 0.05, 0.2, 0.05, 0.05, 0.05]),
    #     viewport=True,
    #     enable_ros2_pub=True,
    #     uw_img_topic="/oceansim/rov/stereo"
    # )

    # Example 5: Get camera pose (after initialization and during simulation)
    print("\n[5] Camera pose tracking...")
    print("   (Call stereo_cam.get_camera_pose() during simulation to get current pose)")

    print("\n" + "=" * 60)
    print("Example completed!")
    print("=" * 60)
    print("\nTo use in your ROV simulation:")
    print("1. Set _use_stereo_camera = True in ui_builder.py")
    print("2. Adjust config/stereo_camera.yaml for your ROV specs")
    print("3. The camera will automatically follow the ROV movement")
    print("4. Subscribe to ROS2 topics for left/right images")


if __name__ == "__main__":
    main()
