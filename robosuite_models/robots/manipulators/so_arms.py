import numpy as np
from robosuite.models.robots.manipulators.manipulator_model import ManipulatorModel
from robosuite.robots import register_robot_class

from robosuite_models import robosuite_model_path_completion

@register_robot_class("FixedBaseRobot")
class SO101(ManipulatorModel):
    """
    Arx5 is a single-arm robot, typically for customizable mounting on quadruped.

    Args:
        idn (int or str): Number or some other unique identification string for this robot instance
    """

    arms = ["right"]

    def __init__(self, idn=0):
        super().__init__(robosuite_model_path_completion("robots/so_arm/so101.xml"), idn=idn)

        # Set joint damping
        self.set_joint_attribute(attrib="damping", values=np.array((0.1, 0.1, 0.1, 0.1, 0.01)))

    @property
    def default_base(self):
        return "NullMount"

    @property
    def default_gripper(self):
        return {"right": "SO101Gripper"}

    @property
    def default_controller_config(self):
        return {"right": "default_so101"}

    @property
    def init_qpos(self):
        return np.array([0.0, -1.57, 1.57, 0, 0.0])

    @property
    def base_xpos_offset(self):
        return {
            "bins": (-0.5, -0.1, 0),
            "empty": (-0.6, 0, 0),
            "table": lambda table_length: (-table_length / 2, 0.0, 0.8),
        }

    @property
    def top_offset(self):
        return np.array((0, 0, 1.0))

    @property
    def _horizontal_radius(self):
        return 0.5

    @property
    def arm_type(self):
        return "single"