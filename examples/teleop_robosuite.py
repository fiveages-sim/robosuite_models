"""Unified teleoperation and data collection script using draccus configuration.

This script combines teleoperation and data collection functionality. Use the 
--collection.enabled flag to enable data collection during teleoperation.

Usage examples:
    # Basic teleoperation only
    python teleop_robosuite.py --env.environment Lift --device.type spacemouse

    # Teleoperation with data collection
    python teleop_robosuite.py --env.environment Lift --device.type spacemouse --collection.enabled true

    # Two-arm environment with data collection
    python teleop_robosuite.py --env.environment TwoArmLift --env.robots Baxter --env.config bimanual --collection.enabled true

    # Custom settings with data collection
    python teleop_robosuite.py --device.pos_sensitivity 2.0 --collection.enabled true --collection.directory /path/to/demos
"""

import datetime
import json
import os
import shutil
import time
from copy import deepcopy
from glob import glob

import draccus
import h5py
import numpy as np

import robosuite as suite
from robosuite import load_composite_controller_config
from robosuite.controllers.composite.composite_controller import WholeBody
from robosuite.wrappers import VisualizationWrapper, DataCollectionWrapper

from config import TeleopConfig


def gather_demonstrations_as_hdf5(directory, out_dir, env_info):
    """
    Gathers the demonstrations saved in @directory into a single hdf5 file.
    
    Args:
        directory (str): Path to the directory containing raw demonstrations.
        out_dir (str): Path to where to store the hdf5 file.
        env_info (str): JSON-encoded string containing environment information.
    """
    hdf5_path = os.path.join(out_dir, "demo.hdf5")
    f = h5py.File(hdf5_path, "w")

    # store some metadata in the attributes of one group
    grp = f.create_group("data")

    num_eps = 0
    env_name = None  # will get populated at some point

    for ep_directory in os.listdir(directory):
        state_paths = os.path.join(directory, ep_directory, "state_*.npz")
        states = []
        actions = []
        success = False

        for state_file in sorted(glob(state_paths)):
            dic = np.load(state_file, allow_pickle=True)
            env_name = str(dic["env"])

            states.extend(dic["states"])
            for ai in dic["action_infos"]:
                actions.append(ai["actions"])
            success = success or dic["successful"]

        if len(states) == 0:
            continue

        # Add only the successful demonstration to dataset
        if success:
            print("Demonstration is successful and has been saved")
            # Delete the last state. This is because when the DataCollector wrapper
            # recorded the states and actions, the states were recorded AFTER playing that action,
            # so we end up with an extra state at the end.
            del states[-1]
            assert len(states) == len(actions)

            num_eps += 1
            ep_data_grp = grp.create_group("demo_{}".format(num_eps))

            # store model xml as an attribute
            xml_path = os.path.join(directory, ep_directory, "model.xml")
            with open(xml_path, "r") as f:
                xml_str = f.read()
            ep_data_grp.attrs["model_file"] = xml_str

            # write datasets for states and actions
            ep_data_grp.create_dataset("states", data=np.array(states))
            ep_data_grp.create_dataset("actions", data=np.array(actions))
        else:
            print("Demonstration is unsuccessful and has NOT been saved")

    # write dataset attributes (metadata)
    now = datetime.datetime.now()
    grp.attrs["date"] = "{}-{}-{}".format(now.month, now.day, now.year)
    grp.attrs["time"] = "{}:{}:{}".format(now.hour, now.minute, now.second)
    grp.attrs["repository_version"] = suite.__version__
    grp.attrs["env"] = env_name
    grp.attrs["env_info"] = env_info

    f.close()


def setup_environment(env_config, control_config, render_config, collection_config):
    """Setup the robosuite environment based on configuration."""
    # Handle robots parameter - convert to list if string
    robots = env_config.robots if isinstance(env_config.robots, list) else [env_config.robots]

    # Get controller config
    controller_config = load_composite_controller_config(
        controller=control_config.controller,
        robot=robots[0],
    )

    if controller_config["type"] == "WHOLE_BODY_MINK_IK":
        # mink-specific import. requires installing mink
        from robosuite.examples.third_party_controller.mink_controller import WholeBodyMinkIK

    # Create argument configuration
    config = {
        "env_name": env_config.environment,
        "robots": robots,
        "controller_configs": controller_config
    }

    # Check if we're using a multi-armed environment and use env_configuration argument if so
    if "TwoArm" in env_config.environment:
        config["env_configuration"] = env_config.config
    
    if env_config.translucent_robot:
        config["translucent_robot"] = True

    # Create environment
    env = suite.make(
        **config,
        has_renderer=render_config.has_renderer,
        has_offscreen_renderer=render_config.has_offscreen_renderer,
        render_camera=render_config.camera,
        ignore_done=True,
        use_camera_obs=render_config.use_camera_obs,
        reward_shaping=True,
        control_freq=20,
    )

    # Wrap this environment in a visualization wrapper
    env = VisualizationWrapper(env, indicator_configs=None)

    # Setup data collection if enabled
    tmp_directory = None
    if collection_config.enabled:
        tmp_directory = "/tmp/{}".format(str(time.time()).replace(".", "_"))
        env = DataCollectionWrapper(env, tmp_directory)

    return env, config, tmp_directory


def setup_device(device_config, env):
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


def teleop_loop(env, device, env_config, render_config, collection_config, tmp_directory, env_info):
    """Main teleoperation loop with optional data collection."""
    # Setup data collection directory if enabled
    if collection_config.enabled:
        if collection_config.directory is None:
            # Use default directory with timestamp when none specified
            t1, t2 = str(time.time()).split(".")
            base_dir = os.path.join(suite.models.assets_root, "demonstrations_private")
            new_dir = os.path.join(base_dir, "{}_{}".format(t1, t2))
        else:
            # Use specified directory directly when provided
            new_dir = collection_config.directory
        
        # Remove existing directory if it has content
        if os.path.exists(new_dir) and os.listdir(new_dir):
            print(f"Directory {new_dir} exists and has content. Removing it...")
            shutil.rmtree(new_dir)
        
        os.makedirs(new_dir, exist_ok=True)
        print(f"Data collection enabled. Saving to: {new_dir}")

    while True:
        # Reset the environment
        env.reset()
        env.render()

        task_completion_hold_count = -1  # counter for data collection

        # Initialize device control
        device.start_control()

        for robot in env.robots:
            robot.print_action_info_dict()

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
            input_ac_dict = device.input2action(mirror_actions=env_config.mirror_actions)

            # If action is none, then this a reset so we should break
            if input_ac_dict is None:
                break

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

            # Handle data collection task completion check
            if collection_config.enabled:
                # Also break if we complete the task (for data collection)
                if task_completion_hold_count == 0:
                    break

                # state machine to check for having a success for 10 consecutive timesteps
                if env._check_success():
                    if task_completion_hold_count > 0:
                        task_completion_hold_count -= 1  # latched state, decrement count
                    else:
                        task_completion_hold_count = 10  # reset count on first success timestep
                else:
                    task_completion_hold_count = -1  # null the counter if there's no success

            # limit frame rate if necessary
            if render_config.max_fr is not None:
                elapsed = time.time() - start
                diff = 1 / render_config.max_fr - elapsed
                if diff > 0:
                    time.sleep(diff)

        # cleanup for end of data collection episodes
        if collection_config.enabled:
            env.close()
            gather_demonstrations_as_hdf5(tmp_directory, new_dir, env_info)


@draccus.wrap()
def main(cfg: TeleopConfig):
    """Main function with draccus configuration."""
    print("Unified Teleoperation & Data Collection Configuration:")
    print(f"  Environment: {cfg.env.environment}")
    print(f"  Robots: {cfg.env.robots}")
    print(f"  Device: {cfg.device.type}")
    print(f"  Controller: {cfg.control.controller}")
    print(f"  Data Collection: {'Enabled' if cfg.collection.enabled else 'Disabled'}")
    if cfg.collection.enabled:
        print(f"  Collection Directory: {cfg.collection.directory}")
    print()

    # Setup printing options for numbers
    np.set_printoptions(formatter={"float": lambda x: "{0:0.3f}".format(x)})

    # Setup environment
    env, env_config_dict, tmp_directory = setup_environment(cfg.env, cfg.control, cfg.render, cfg.collection)

    # Convert environment config to json for data collection
    env_info = json.dumps(env_config_dict) if cfg.collection.enabled else None

    # Setup device
    device = setup_device(cfg.device, env)

    try:
        # Start teleoperation loop (with optional data collection)
        teleop_loop(env, device, cfg.env, cfg.render, cfg.collection, tmp_directory, env_info)
    except KeyboardInterrupt:
        print("\nTeleoperation interrupted by user.")
    finally:
        if hasattr(device, 'disconnect'):
            device.disconnect()


if __name__ == "__main__":
    main()
