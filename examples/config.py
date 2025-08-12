"""Configuration classes for robosuite teleoperation and data collection.

This module contains all configuration classes used across different robosuite scripts
for consistent parameter management using draccus.
"""

from dataclasses import dataclass, field
from typing import List, Optional, Union

from lerobot.teleoperators import TeleoperatorConfig


@dataclass
class EnvironmentConfig:
    """Environment-related configuration."""
    environment: str = "Lift"
    mirror_actions: bool = False
    robots: Union[str, List[str]] = "Panda"
    config: str = "default"
    arm: str = "right"
    switch_on_grasp: bool = False
    toggle_camera_on_grasp: bool = False
    translucent_robot: bool = False


@dataclass
class DeviceConfig:
    """Device-related configuration."""
    teleoperator: TeleoperatorConfig = field(default_factory=TeleoperatorConfig)
    type: str = "spacemouse"  # spacemouse, keyboard, dualsense, mjgui, lerobot_lead, ros2_joy
    pos_sensitivity: float = 1.0
    rot_sensitivity: float = 1.0
    reverse_xy: bool = False
    joy_topic: str = "/joy"  # ROS2 joy topic name
    mapping_config: Optional[dict] = None  # Custom mapping configuration for ROS2 Joy


@dataclass
class ControllerConfig:
    """Controller-related configuration."""
    controller: Optional[str] = None


@dataclass
class RenderConfig:
    """Rendering and performance configuration."""
    max_fr: int = 20
    renderer: str = "mjviewer"  # mjviewer, mujoco
    camera: Union[str, List[str]] = "agentview"
    has_renderer: bool = True
    has_offscreen_renderer: bool = False
    use_camera_obs: bool = False


@dataclass
class CollectionConfig:
    """Data collection specific configuration."""
    enabled: bool = False  # Whether to enable data collection
    directory: str = None


@dataclass
class TeleopConfig:
    """Main teleop configuration."""
    env: EnvironmentConfig = field(default_factory=EnvironmentConfig)
    device: DeviceConfig = field(default_factory=DeviceConfig)
    control: ControllerConfig = field(default_factory=ControllerConfig)
    render: RenderConfig = field(default_factory=RenderConfig)
    collection: CollectionConfig = field(default_factory=CollectionConfig)


@dataclass
class CollectDemosConfig:
    """Configuration for collecting demonstrations."""
    env: EnvironmentConfig = field(default_factory=EnvironmentConfig)
    device: DeviceConfig = field(default_factory=DeviceConfig)
    control: ControllerConfig = field(default_factory=ControllerConfig)
    render: RenderConfig = field(default_factory=RenderConfig)
    collection: CollectionConfig = field(default_factory=CollectionConfig)
