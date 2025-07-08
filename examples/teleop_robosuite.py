"""Teleoperate robot with keyboard or SpaceMouse using draccus configuration.

This is a refactored version of teleop_robosuite.py that uses draccus + dataclass
for configuration management instead of argparse.

Usage examples:
    # Basic usage
    python teleop_robosuite_draccus.py --env.environment Lift --device.type spacemouse

    # Two-arm environment
    python teleop_robosuite_draccus.py --env.environment TwoArmLift --env.robots Baxter --env.config bimanual --env.arm left

    # Custom sensitivity
    python teleop_robosuite_draccus.py --device.pos_sensitivity 2.0 --device.rot_sensitivity 1.5
"""

import time
from dataclasses import dataclass, field
from typing import List, Optional, Union

import draccus
import numpy as np
from lerobot.teleoperators import (so101_leader, TeleoperatorConfig)

import robosuite as suite
from robosuite import load_composite_controller_config
from robosuite.controllers.composite.composite_controller import WholeBody
from robosuite.wrappers import VisualizationWrapper


@dataclass
class EnvironmentConfig:
    """Environment-related configuration."""
    environment: str = "Lift"
    robots: Union[str, List[str]] = "Panda"
    config: str = "default"
    arm: str = "right"
    switch_on_grasp: bool = False
    toggle_camera_on_grasp: bool = False


@dataclass
class DeviceConfig:
    """Device-related configuration."""
    teleoperator: TeleoperatorConfig = field(default_factory=TeleoperatorConfig)
    type: str = "spacemouse"  # spacemouse, keyboard, dualsense, mjgui
    pos_sensitivity: float = 1.0
    rot_sensitivity: float = 1.0
    reverse_xy: bool = False


@dataclass
class ControllerConfig:
    """Controller-related configuration."""
    controller: Optional[str] = None


@dataclass
class RenderConfig:
    """Rendering and performance configuration."""
    max_fr: int = 20


@dataclass
class TeleopConfig:
    """Main teleop configuration."""
    env: EnvironmentConfig = field(default_factory=EnvironmentConfig)
    device: DeviceConfig = field(default_factory=DeviceConfig)
    control: ControllerConfig = field(default_factory=ControllerConfig)
    render: RenderConfig = field(default_factory=RenderConfig)


def setup_environment(env_config: EnvironmentConfig, control_config: ControllerConfig):
    """Setup the robosuite environment based on configuration."""
    # Handle robots parameter - convert to list if string
    robots = env_config.robots if isinstance(env_config.robots, list) else [env_config.robots]

    # Get controller config
    controller_config = load_composite_controller_config(
        controller=control_config.controller,
        robot=robots[0],
    )

    # Create argument configuration
    config = {
        "env_name": env_config.environment,
        "robots": robots,
        "controller_configs": controller_config,
    }

    # Check if we're using a multi-armed environment and use env_configuration argument if so
    if "TwoArm" in env_config.environment:
        config["env_configuration"] = env_config.config

    # Create environment
    env = suite.make(
        **config,
        has_renderer=True,
        has_offscreen_renderer=False,
        render_camera="agentview",
        ignore_done=True,
        use_camera_obs=False,
        reward_shaping=True,
        control_freq=20,
        hard_reset=False,
    )

    # Wrap this environment in a visualization wrapper
    env = VisualizationWrapper(env, indicator_configs=None)

    return env


def setup_device(device_config: DeviceConfig, env):
    """Setup the input device based on configuration."""
    if device_config.type == "keyboard":
        from robosuite.devices import Keyboard
        device = Keyboard(
            env=env,
            pos_sensitivity=device_config.pos_sensitivity,
            rot_sensitivity=device_config.rot_sensitivity,
        )
        env.viewer.add_keypress_callback(device.on_press)
    elif device_config.type == "spacemouse":
        from robosuite.devices import SpaceMouse
        device = SpaceMouse(
            env=env,
            pos_sensitivity=device_config.pos_sensitivity,
            rot_sensitivity=device_config.rot_sensitivity,
        )
    elif device_config.type == "dualsense":
        from robosuite.devices import DualSense
        device = DualSense(
            env=env,
            pos_sensitivity=device_config.pos_sensitivity,
            rot_sensitivity=device_config.rot_sensitivity,
            reverse_xy=device_config.reverse_xy,
        )
    elif device_config.type == "mjgui":
        from robosuite.devices.mjgui import MJGUI
        device = MJGUI(env=env)
    elif device_config.type == "lerobot_lead":
        from robosuite.devices.lerobot_lead import LeRobotLead
        device = LeRobotLead(env=env, teleoperator=device_config.teleoperator)
    else:
        raise ValueError(
            f"Invalid device choice: {device_config.type}. Choose 'keyboard', 'dualsense', 'spacemouse', or 'mjgui'.")

    return device


def teleop_loop(env, device, env_config: EnvironmentConfig, render_config: RenderConfig):
    """Main teleoperation loop."""
    while True:
        # Reset the environment
        obs = env.reset()

        # Setup rendering
        cam_id = 0
        num_cam = len(env.sim.model.camera_names)
        env.render()

        # Initialize variables that should the maintained between resets
        last_grasp = 0

        # Initialize device control
        device.start_control()
        all_prev_gripper_actions = [
            {
                f"{robot_arm}_gripper": np.repeat([0], robot.gripper[robot_arm].dof)
                for robot_arm in robot.arms
                if robot.gripper[robot_arm].dof > 0
            }
            for robot in env.robots
        ]

        # Loop until we get a reset from the input or the task completes
        while True:
            start = time.time()

            # Set active robot
            active_robot = env.robots[device.active_robot]

            # Get the newest action
            input_ac_dict = device.input2action()

            # If action is none, then this a reset so we should break
            if input_ac_dict is None:
                break

            from copy import deepcopy
            action_dict = deepcopy(input_ac_dict)

            # set arm actions
            for arm in active_robot.arms:
                if isinstance(active_robot.composite_controller, WholeBody):
                    controller_input_type = active_robot.composite_controller.joint_action_policy.input_type
                else:
                    controller_input_type = active_robot.part_controllers[arm].input_type

                if controller_input_type == "delta":
                    action_dict[arm] = input_ac_dict[f"{arm}_delta"]
                elif controller_input_type == "absolute":
                    action_dict[arm] = input_ac_dict[f"{arm}_abs"]
                else:
                    raise ValueError(f"Unknown controller input type: {controller_input_type}")

            # Maintain gripper state for each robot but only update the active robot with action
            env_action = [robot.create_action_vector(all_prev_gripper_actions[i]) for i, robot in enumerate(env.robots)]
            env_action[device.active_robot] = active_robot.create_action_vector(action_dict)
            env_action = np.concatenate(env_action)
            for gripper_ac in all_prev_gripper_actions[device.active_robot]:
                all_prev_gripper_actions[device.active_robot][gripper_ac] = action_dict[gripper_ac]

            env.step(env_action)
            env.render()

            # limit frame rate if necessary
            if render_config.max_fr is not None:
                elapsed = time.time() - start
                diff = 1 / render_config.max_fr - elapsed
                if diff > 0:
                    time.sleep(diff)


@draccus.wrap()
def main(cfg: TeleopConfig):
    """Main function with draccus configuration."""
    print("Teleoperation Configuration:")
    print(f"  Environment: {cfg.env.environment}")
    print(f"  Robots: {cfg.env.robots}")
    print(f"  Device: {cfg.device.type}")
    print(f"  Controller: {cfg.control.controller}")
    print()

    # Setup printing options for numbers
    np.set_printoptions(formatter={"float": lambda x: "{0:0.3f}".format(x)})

    # Setup environment
    env = setup_environment(cfg.env, cfg.control)

    # Setup device
    device = setup_device(cfg.device, env)

    try:
        # Start teleoperation loop
        teleop_loop(env, device, cfg.env, cfg.render)
    except KeyboardInterrupt:
        print("\nTeleoperation interrupted by user.")
    finally:
        if hasattr(device, 'disconnect'):
            device.disconnect()


if __name__ == "__main__":
    main()
