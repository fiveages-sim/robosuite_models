"""
Grippers for ARX R5/X5/X7/X7s
"""

import numpy as np
from robosuite.models.grippers import register_gripper
from robosuite.models.grippers.gripper_model import GripperModel

from robosuite_models import robosuite_model_path_completion


@register_gripper
class SO101GripperBase(GripperModel):
    """
    Gripper for ARX R5/X5 gripper (has two fingers).

    Args:
        idn (int or str): Number or some other unique identification string for this gripper instance
    """

    def __init__(self, idn=0):
        super().__init__(robosuite_model_path_completion("grippers/so_arm_gripper/so101_gripper.xml"), idn=idn)

    def format_action(self, action):
        return action

    @property
    def init_qpos(self):
        return np.array([0.])

    @property
    def _important_geoms(self):
        return {
            "left_finger": [
                "gripper_collision_0",
                "gripper_collision_1",
                "gripper_collision_2",
            ],
            "right_finger": [
                "moving_jaw_so101_v1_collision_0",
                "moving_jaw_so101_v1_collision_1",
                "moving_jaw_so101_v1_collision_2",
            ],
            "left_fingerpad": ["fixed_jaw_pad_collision"],
            "right_fingerpad": ["moving_jaw_pad_collision"],
        }


@register_gripper
class SO101Gripper(SO101GripperBase):
    """
    Modifies ArxGripperBase to only take one action.
    """

    def format_action(self, action):
        """
        Maps continuous action into binary output
        -1 => open, 1 => closed

        Args:
            action (np.array): gripper-specific action

        Raises:
            AssertionError: [Invalid action dimension size]
        """
        assert len(action) == self.dof
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