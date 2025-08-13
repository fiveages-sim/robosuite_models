"""
ChangingTek AG2F90-C gripper with parallel jaw mechanism.
"""
import numpy as np
from robosuite.models.grippers import register_gripper
from robosuite.models.grippers.gripper_model import GripperModel

from robosuite_models import robosuite_model_path_completion


@register_gripper
class AG2F90CBase(GripperModel):
    """
    ChangingTek AG2F90-C gripper with parallel jaw mechanism.

    Args:
        idn (int or str): Number or some other unique identification string for this gripper instance
    """

    def __init__(self, idn=0):
        super().__init__(robosuite_model_path_completion("grippers/changingtek/AG2F90-C/gripper.xml"), idn=idn)

    def format_action(self, action):
        return action

    @property
    def init_qpos(self):
        return np.array([0.0])

    @property
    def _important_geoms(self):
        return {
            "left_finger": [
                "left_1_link_collision_0",
                "left_1_link_collision_1", 
                "left_1_link_collision_2",
                "left_in_link_collision_0",
                "left_in_link_collision_1",
                "left_in_link_collision_2",
                "left_up_link_collision_0",
                "left_up_link_collision_1",
                "left_up_link_collision_2",
                "left_2_link_collision_0",
                "left_2_link_collision_1",
                "left_2_link_collision_2",
            ],
            "right_finger": [
                "right_1_link_collision_0",
                "right_1_link_collision_1",
                "right_1_link_collision_2",
                "right_in_link_collision_0",
                "right_in_link_collision_1",
                "right_in_link_collision_2",
                "right_up_link_collision_0",
                "right_up_link_collision_1",
                "right_up_link_collision_2",
                "right_2_link_collision_0",
                "right_2_link_collision_1",
                "right_2_link_collision_2",
            ],
            "left_fingerpad": [
                "left_Pad_link_collision_0",
                "left_Pad_link_collision_1",
                "left_Pad_link_collision_2",
            ],
            "right_fingerpad": [
                "right_Pad_link_collision_0",
                "right_Pad_link_collision_1",
                "right_Pad_link_collision_2",
            ],
        }


@register_gripper
class AG2F90C(AG2F90CBase):
    """
    1-DoF variant of ChangingTekAG2F90CGripperBase.
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
