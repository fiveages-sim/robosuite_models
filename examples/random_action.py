import time
import argparse

from robosuite.robots import MobileRobot
from robosuite.utils.input_utils import *
import robosuite as suite
import numpy as np

MAX_FR = 25  # max frame rate for running simluation

if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("--environment", type=str, default="Lift")
    parser.add_argument(
        "--robots", nargs="+", type=str, default="ArxR5", help="Which robot(s) to use in the env"
    )

    args = parser.parse_args()
    config = {
        "env_name": args.environment,
        "robots": args.robots,
    }


    # initialize the task
    env = suite.make(
        **config,
        has_renderer=True,
        has_offscreen_renderer=False,
        ignore_done=True,
        use_camera_obs=False,
        control_freq=20,
    )
    env.reset()
    env.viewer.set_camera(camera_id=0)
    for robot in env.robots:
        if isinstance(robot, MobileRobot):
            robot.enable_parts(legs=False, base=False)

    # do visualization
    for i in range(10000):
        start = time.time()
        action = np.random.randn(*env.action_spec[0].shape)
        obs, reward, done, _ = env.step(action)
        env.render()

        # limit frame rate if necessary
        elapsed = time.time() - start
        diff = 1 / MAX_FR - elapsed
        if diff > 0:
            time.sleep(diff)
