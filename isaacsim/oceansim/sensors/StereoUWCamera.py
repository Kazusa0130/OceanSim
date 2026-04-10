# Stereo Underwater Camera Sensor for OceanSim
import omni.ui as ui
import numpy as np
import yaml
import carb
from pathlib import Path

from isaacsim.oceansim.sensors.UW_Camera import UW_Camera


class StereoUWCamera:
    """Stereo underwater camera consisting of two UW_Camera instances.

    This class creates a stereo camera pair with a configurable baseline,
    allowing for stereo vision underwater simulation.

    Attributes:
        left_cam (UW_Camera): Left camera of the stereo pair
        right_cam (UW_Camera): Right camera of the stereo pair
        baseline (float): Distance between left and right camera centers (in meters)
        name (str): Name of the stereo camera sensor
    """

    def __init__(self,
                 prim_path_prefix: str,
                 name: str = "StereoUWCamera",
                 frequency: float = None,
                 dt: float = None,
                 resolution: tuple = None,
                 position: np.ndarray = None,
                 orientation: np.ndarray = None,
                 translation: np.ndarray = None,
                 baseline: float = 0.1,
                 focal_length: float = 2.1,
                 clipping_range: tuple = (0.1, 100.0),
                 yaml_config_path: str = None):
        """Initialize stereo underwater camera.

        Args:
            prim_path_prefix (str): Base prim path (e.g., '/World/rob/stereo_camera')
            name (str, optional): Name of the stereo camera. Defaults to "StereoUWCamera".
            frequency (float, optional): Update frequency in Hz. Defaults to None.
            dt (float, optional): Update period in seconds. Defaults to None.
            resolution (tuple, optional): Camera resolution (width, height). Defaults to None.
            position (np.ndarray, optional): World frame position. Defaults to None.
            orientation (np.ndarray, optional): Quaternion orientation (w,x,y,z). Defaults to None.
            translation (np.ndarray, optional): Local frame translation. Defaults to None.
            baseline (float, optional): Camera baseline in meters. Defaults to 0.1.
            focal_length (float, optional): Camera focal length. Defaults to 2.1.
            clipping_range (tuple, optional): Near and far clipping planes. Defaults to (0.1, 100.0).
            yaml_config_path (str, optional): Path to YAML config file. If provided, other params are overridden.
        """
        self._name = name
        self._prim_path_prefix = prim_path_prefix

        # Load config from YAML if provided
        if yaml_config_path is not None and yaml_config_path != '':
            config = self._load_yaml_config(yaml_config_path)
            baseline = config.get('baseline', baseline)
            frequency = config.get('frequency', frequency)
            dt = config.get('dt', dt)
            resolution = config.get('resolution', resolution)
            focal_length = config.get('focal_length', focal_length)
            clipping_range = config.get('clipping_range', clipping_range)
            if translation is None:
                translation = config.get('translation', None)
            if orientation is None:
                orientation = config.get('orientation', None)
        else:
            print(f'[{self._name}] No YAML config provided. Using default parameters: baseline={baseline}m')

        self._baseline = baseline
        self._focal_length = focal_length
        self._clipping_range = clipping_range
        self._resolution = resolution
        self._frequency = frequency
        self._orientation = orientation

        # Calculate left and right camera translations based on baseline
        # Left camera is at -baseline/2 in Y, right camera at +baseline/2 in Y
        # Assuming cameras are separated horizontally (along Y axis in local frame)
        left_translation = self._calculate_camera_translation(translation, -baseline / 2.0)
        right_translation = self._calculate_camera_translation(translation, baseline / 2.0)

        # Create left camera
        self.left_cam = UW_Camera(
            prim_path=prim_path_prefix + '_left',
            name=name + '_Left',
            frequency=frequency,
            dt=dt,
            resolution=resolution,
            position=position,
            orientation=orientation,
            translation=left_translation
        )

        # Create right camera
        self.right_cam = UW_Camera(
            prim_path=prim_path_prefix + '_right',
            name=name + '_Right',
            frequency=frequency,
            dt=dt,
            resolution=resolution,
            position=position,
            orientation=orientation,
            translation=right_translation
        )

        print(f'[{self._name}] Stereo camera created with baseline: {baseline}m, focal_length: {focal_length}mm')

    def _load_yaml_config(self, yaml_path: str) -> dict:
        """Load configuration from YAML file.

        Args:
            yaml_path (str): Path to YAML configuration file

        Returns:
            dict: Configuration dictionary
        """
        config = {}
        if yaml_path is None or yaml_path == '':
            return config

        try:
            with open(yaml_path, 'r') as file:
                yaml_content = yaml.safe_load(file)
                if yaml_content and 'stereo_camera' in yaml_content:
                    config = yaml_content['stereo_camera']
                    print(f'[{self._name}] Loaded configuration from {yaml_path}')
                else:
                    carb.log_warn(f'[{self._name}] No "stereo_camera" section found in {yaml_path}')
        except FileNotFoundError:
            carb.log_warn(f'[{self._name}] YAML config file not found: {yaml_path}. Using default parameters.')
        except yaml.YAMLError as exc:
            carb.log_error(f'[{self._name}] Error parsing YAML file: {exc}')

        return config

    def _calculate_camera_translation(self, base_translation: np.ndarray, y_offset: float) -> np.ndarray:
        """Calculate camera translation with Y-axis offset for stereo baseline.

        Args:
            base_translation (np.ndarray): Base translation vector
            y_offset (float): Offset along Y axis (baseline component)

        Returns:
            np.ndarray: Translation vector with offset applied
        """
        if base_translation is None:
            return np.array([0.0, y_offset, 0.0])
        else:
            translation = np.array(base_translation, dtype=float)
            translation[1] += y_offset  # Apply offset to Y axis
            return translation

    def initialize(self,
                   UW_param: np.ndarray = np.array([0.0, 0.31, 0.24, 0.05, 0.05, 0.2, 0.05, 0.05, 0.05]),
                   viewport: bool = True,
                   writing_dir: str = None,
                   UW_yaml_path: str = None,
                   physics_sim_view=None,
                   enable_ros2_pub: bool = True,
                   uw_img_topic: str = None,
                   ros2_pub_frequency: int = 5,
                   ros2_pub_jpeg_quality: int = 50):
        """Initialize both cameras with underwater rendering parameters.

        Args:
            UW_param (np.ndarray, optional): Underwater parameters array. Defaults to None.
            viewport (bool, optional): Enable viewport visualization. Defaults to True.
            writing_dir (str, optional): Directory to save images. Defaults to None.
            UW_yaml_path (str, optional): Path to YAML file with water properties. Defaults to None.
            physics_sim_view: Physics simulation view
            enable_ros2_pub (bool, optional): Enable ROS2 communication. Defaults to True.
            uw_img_topic (str, optional): ROS2 topic base name. Defaults to None.
            ros2_pub_frequency (int, optional): ROS2 publish frequency. Defaults to 5.
            ros2_pub_jpeg_quality (int, optional): ROS2 JPEG quality. Defaults to 50.
        """
        # Use default UW_param if None
        if UW_param is None:
            UW_param = np.array([0.0, 0.31, 0.24, 0.05, 0.05, 0.2, 0.05, 0.05, 0.05])

        # Set focal length and clipping range for both cameras
        self.left_cam.set_focal_length(self._focal_length)
        self.left_cam.set_clipping_range(*self._clipping_range)
        self.right_cam.set_focal_length(self._focal_length)
        self.right_cam.set_clipping_range(*self._clipping_range)

        # Create shared ROS2 node for stereo camera to avoid resource conflicts
        shared_ros2_node = None
        if enable_ros2_pub:
            try:
                import rclpy
                if not rclpy.ok():
                    rclpy.init()
                node_name = f'oceansim_stereo_{self._name.lower()}_pub'.replace(' ', '_')
                shared_ros2_node = rclpy.create_node(node_name)
                print(f'[{self._name}] Shared ROS2 node created: {node_name}')
            except Exception as e:
                print(f'[{self._name}] Failed to create shared ROS2 node: {e}')
                enable_ros2_pub = False

        # Initialize left camera
        left_topic = f"{uw_img_topic}/left" if uw_img_topic else "/oceansim/robot/stereo/left"
        self.left_cam.initialize(
            UW_param=UW_param,
            viewport=viewport,
            writing_dir=writing_dir + "/left" if writing_dir else None,
            UW_yaml_path=UW_yaml_path,
            physics_sim_view=physics_sim_view,
            enable_ros2_pub=enable_ros2_pub,
            uw_img_topic=left_topic,
            ros2_pub_frequency=ros2_pub_frequency,
            ros2_pub_jpeg_quality=ros2_pub_jpeg_quality,
            ros2_node=shared_ros2_node
        )

        # Initialize right camera
        right_topic = f"{uw_img_topic}/right" if uw_img_topic else "/oceansim/robot/stereo/right"
        self.right_cam.initialize(
            UW_param=UW_param,
            viewport=viewport,
            writing_dir=writing_dir + "/right" if writing_dir else None,
            UW_yaml_path=UW_yaml_path,
            physics_sim_view=physics_sim_view,
            enable_ros2_pub=enable_ros2_pub,
            uw_img_topic=right_topic,
            ros2_pub_frequency=ros2_pub_frequency,
            ros2_pub_jpeg_quality=ros2_pub_jpeg_quality,
            ros2_node=shared_ros2_node
        )

        # Store shared node for cleanup
        self._shared_ros2_node = shared_ros2_node

        print(f'[{self._name}] Both cameras initialized successfully')

    def render(self):
        """Render both left and right camera frames."""
        self.left_cam.render()
        self.right_cam.render()

    def close(self):
        """Clean up resources for both cameras."""
        self.left_cam.close()
        self.right_cam.close()

        # Clean up shared ROS2 node
        if hasattr(self, '_shared_ros2_node') and self._shared_ros2_node is not None:
            try:
                self._shared_ros2_node.destroy_node()
                print(f'[{self._name}] Shared ROS2 node destroyed')
            except Exception as e:
                print(f'[{self._name}] Error destroying shared ROS2 node: {e}')

        print(f'[{self._name}] Stereo camera resources released')

    def get_baseline(self) -> float:
        """Get the stereo camera baseline.

        Returns:
            float: Baseline distance in meters
        """
        return self._baseline

    def set_baseline(self, baseline: float):
        """Set a new baseline (requires scene reload to take effect).

        Args:
            baseline (float): New baseline in meters
        """
        self._baseline = baseline
        print(f'[{self._name}] Baseline updated to {baseline}m. Reload scene to apply.')

    def get_resolution(self) -> tuple:
        """Get camera resolution.

        Returns:
            tuple: (width, height) resolution
        """
        return self._resolution if self._resolution else (1920, 1080)

    def get_extrinsics(self) -> dict:
        """Get camera extrinsics (transformation relative to parent/rob).

        Returns:
            dict: Dictionary containing translation and orientation
        """
        return {
            'translation': self.left_cam._translation if hasattr(self.left_cam, '_translation') else None,
            'orientation': self._orientation,
            'baseline': self._baseline
        }
