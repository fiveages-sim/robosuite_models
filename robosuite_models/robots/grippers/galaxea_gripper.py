"""
Galaxea G1 gripper with parallel jaw mechanism.
"""
import numpy as np
from robosuite.models.grippers import register_gripper
from robosuite.models.grippers.gripper_model import GripperModel

from robosuite_models import robosuite_model_path_completion


@register_gripper
class GalaxeaG1Base(GripperModel):
    """
    Galaxea G1 gripper with parallel jaw mechanism.

    Args:
        idn (int or str): Number or some other unique identification string for this gripper instance
    """

    def __init__(self, idn=0):
        super().__init__(robosuite_model_path_completion("grippers/galaxea/g1_gripper/gripper.xml"), idn=idn)

    def format_action(self, action):
        return action

    @property
    def init_qpos(self):
        return np.array([0.0])

    @property
    def _important_geoms(self):
        return {
            "left_finger": ["finger_collision_1", "finger_collision_2"],
            "right_finger": ["finger_1_collision_1", "finger_1_collision_2"],
            "left_fingerpad": ["finger1_pad_collision"],
            "right_fingerpad": ["finger2_pad_collision"],
        }


@register_gripper
class GalaxeaG1(GalaxeaG1Base):

    def format_action(self, action):
        assert len(action) == 1
        self.current_action = np.clip(
            self.current_action + np.array([-1.0]) * self.speed * np.sign(action), -1.0, 1.0
        )
        return self.current_action

    @property
    def speed(self):
        return 0.2

    @property
    def dof(self):
        return 1
