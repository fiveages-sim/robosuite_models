"""
Grippers for ARX R5/X5/X7/X7s
"""

import numpy as np
from robosuite.models.grippers import register_gripper
from robosuite.models.grippers.gripper_model import GripperModel

from robosuite_models import robosuite_model_path_completion


@register_gripper
class ArxGripperBase(GripperModel):
    """
    Gripper for ARX R5/X5 gripper (has two fingers).

    Args:
        idn (int or str): Number or some other unique identification string for this gripper instance
    """

    def __init__(self, idn=0):
        super().__init__(robosuite_model_path_completion("grippers/arx/arx_gripper.xml"), idn=idn)

    def format_action(self, action):
        return action

    @property
    def init_qpos(self):
        return np.array([0.0042, 0.0042])

    @property
    def _important_geoms(self):
        return {
            "left_finger": [
                "link7_collision_0",
                "link7_collision_1",
                "link7_collision_2",
                "link7_collision_3",
                "link7_collision_4",
                "link7_collision_5",
            ],
            "right_finger": [
                "link8_collision_0",
                "link8_collision_1",
                "link8_collision_2",
                "link8_collision_3",
                "link8_collision_4",
                "link8_collision_5",
            ],
            "left_fingerpad": ["link7_collision_1"],
            "right_fingerpad": ["link8_collision_1"],
        }


@register_gripper
class ArxGripper(ArxGripperBase):
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
            self.current_action + np.array([-1.0, -1.0]) * self.speed * np.sign(action), -1.0, 1.0
        )
        return self.current_action

    @property
    def speed(self):
        return 0.2

    @property
    def dof(self):
        return 1


@register_gripper
class ArxX7GripperBase(GripperModel):
    """
    Gripper for ARX X7/X7s gripper (has two fingers).

    Args:
        idn (int or str): Number or some other unique identification string for this gripper instance
    """

    def __init__(self, idn=0):
        super().__init__(robosuite_model_path_completion("grippers/arx/arx_x7_gripper.xml"), idn=idn)

    def format_action(self, action):
        return action

    @property
    def init_qpos(self):
        return np.array([0.0042, 0.0042])

    @property
    def _important_geoms(self):
        return {
            "left_finger": [
                "gripper_left_collision_0",
                "gripper_left_collision_1",
                "gripper_left_collision_2",
                "gripper_left_collision_3",
                "gripper_left_collision_4",
                "gripper_left_collision_5",
                "gripper_left_collision_6",
                "gripper_left_collision_7",
                "gripper_left_collision_8",
                "gripper_left_collision_9",
                "finger1_pad_collision",
            ],
            "right_finger": [
                "gripper_right_collision_0",
                "gripper_right_collision_1",
                "gripper_right_collision_2",
                "gripper_right_collision_3",
                "gripper_right_collision_4",
                "gripper_right_collision_5",
                "gripper_right_collision_6",
                "gripper_right_collision_7",
                "gripper_right_collision_8",
                "gripper_right_collision_9",
                "finger2_pad_collision",
            ],
            "left_fingerpad": ["finger1_pad_collision"],
            "right_fingerpad": ["finger2_pad_collision"],
        }


@register_gripper
class ArxX7Gripper(ArxX7GripperBase):
    """
    Modifies ArxX7GripperBase to only take one action.
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
            self.current_action + np.array([-1.0, -1.0]) * self.speed * np.sign(action), -1.0, 1.0
        )
        return self.current_action

    @property
    def speed(self):
        return 0.2

    @property
    def dof(self):
        return 1
