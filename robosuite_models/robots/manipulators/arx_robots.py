import numpy as np
from robosuite.models.robots.manipulators.manipulator_model import ManipulatorModel
from robosuite.robots import register_robot_class

from robosuite_models import robosuite_model_path_completion

@register_robot_class("FixedBaseRobot")
class Arx5(ManipulatorModel):
    """
    Arx5 is a single-arm robot, typically for customizable mounting on quadruped.

    Args:
        idn (int or str): Number or some other unique identification string for this robot instance
    """

    arms = ["right"]

    def __init__(self, idn=0):
        super().__init__(robosuite_model_path_completion("robots/arx5/robot.xml"), idn=idn)

        # Set joint damping
        self.set_joint_attribute(attrib="damping", values=np.array((0.1, 0.1, 0.1, 0.1, 0.1, 0.01)))

    @property
    def default_base(self):
        return "RethinkMount"

    @property
    def default_gripper(self):
        return {"right": "UMIGripper"}

    @property
    def default_controller_config(self):
        return {"right": "default_spot"}

    @property
    def init_qpos(self):
        return np.array([0.0, 0.3, 0.7, -0.67, 0.0, 0.12])

    @property
    def base_xpos_offset(self):
        return {
            "bins": (-0.5, -0.1, 0),
            "empty": (-0.6, 0, 0),
            "table": lambda table_length: (-0.35 - table_length / 2, 0.0, 0.0),
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

@register_robot_class("FixedBaseRobot")
class ArxR5(ManipulatorModel):
    """
    Arx R5 is a single-arm robot.

    Args:
        idn (int or str): Number or some other unique identification string for this robot instance
    """

    arms = ["right"]

    def __init__(self, idn=0):
        super().__init__(robosuite_model_path_completion("robots/arx_r5/robot.xml"), idn=idn)

        # Set joint damping
        self.set_joint_attribute(attrib="damping", values=np.array((0.1, 0.1, 0.1, 0.1, 0.1, 0.01)))

    @property
    def default_base(self):
        return "RethinkMount"

    @property
    def default_gripper(self):
        return {"right": "ArxGripper"}

    @property
    def default_controller_config(self):
        return {"right": "default_dobot_cr5"}

    @property
    def init_qpos(self):
        return np.array([0.0, 0.3, 0.7, -0.67, 0.0, 0.12])

    @property
    def base_xpos_offset(self):
        return {
            "bins": (-0.5, -0.1, 0),
            "empty": (-0.6, 0, 0),
            "table": lambda table_length: (-0.35 - table_length / 2, 0.0, 0.0),
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

@register_robot_class("FixedBaseRobot")
class ArxR5Dual(ManipulatorModel):
    """
    Arx R5 Dual Arm configuration. For use with two Arx R5 arms mounted on a single base.

    Args:
        idn (int or str): Number or some other unique identification string for this robot instance
    """

    arms = ["right", "left"]

    def __init__(self, idn=0):
        super().__init__(robosuite_model_path_completion("robots/arx_r5/dual_robot.xml"), idn=idn)

        # Set joint damping
        self.set_joint_attribute(attrib="damping",
                                 values=np.array((0.1, 0.1, 0.1, 0.1, 0.1, 0.01,
                                                  0.1, 0.1, 0.1, 0.1, 0.1, 0.01)))

    @property
    def default_base(self):
        return "NullMount"

    @property
    def default_gripper(self):
        return {
            "right": "ArxGripper",
            "left": "ArxGripper"
        }

    @property
    def default_controller_config(self):
        """
        Since this is bimanual robot, returns dict with `'right'`, `'left'` keywords corresponding to their respective
        values

        Returns:
            dict: Dictionary containing arm-specific default controller config names
        """
        return {
            "right": "default_arxlift",
            "left": "default_arxlift",
        }

    @property
    def init_qpos(self):
        return np.array([0.0, 0.3, 0.7, -0.67, 0.0, 0.12,
                         0.0, 0.3, 0.7, -0.67, 0.0, 0.12])

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
        return "bimanual"

    @property
    def _eef_name(self):
        """
        Since this is bimanual robot, returns dict with `'right'`, `'left'` keywords corresponding to their respective
        values

        Returns:
            dict: Dictionary containing arm-specific eef names
        """
        return {"right": "right_hand", "left": "left_hand"}

@register_robot_class("FixedBaseRobot")
class ArxX5(ManipulatorModel):
    """
    Arx X5 is a single-arm robot.

    Args:
        idn (int or str): Number or some other unique identification string for this robot instance
    """

    arms = ["right"]

    def __init__(self, idn=0):
        super().__init__(robosuite_model_path_completion("robots/arx_x5/robot.xml"), idn=idn)

        # Set joint damping
        self.set_joint_attribute(attrib="damping", values=np.array((0.1, 0.1, 0.1, 0.1, 0.1, 0.01)))

    @property
    def default_base(self):
        return "RethinkMount"

    @property
    def default_gripper(self):
        return {"right": "ArxGripper"}

    @property
    def default_controller_config(self):
        return {"right": "default_dobot_cr5"}

    @property
    def init_qpos(self):
        return np.array([0.0, 0.3, 0.7, -0.67, 0.0, 0.12])

    @property
    def base_xpos_offset(self):
        return {
            "bins": (-0.5, -0.1, 0),
            "empty": (-0.6, 0, 0),
            "table": lambda table_length: (-0.35 - table_length / 2, 0.0, 0.0),
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

@register_robot_class("FixedBaseRobot")
class ArxX5Dual(ManipulatorModel):
    """
    Arx X5 Dual Arm configuration. For use with two Arx X5 arms mounted on a single base.

    Args:
        idn (int or str): Number or some other unique identification string for this robot instance
    """

    arms = ["right", "left"]

    def __init__(self, idn=0):
        super().__init__(robosuite_model_path_completion("robots/arx_x5/dual_robot.xml"), idn=idn)

        # Set joint damping
        self.set_joint_attribute(attrib="damping",
                                 values=np.array((0.1, 0.1, 0.1, 0.1, 0.1, 0.01,
                                                  0.1, 0.1, 0.1, 0.1, 0.1, 0.01)))

    @property
    def default_base(self):
        return "NullMount"

    @property
    def default_gripper(self):
        return {
            "right": "ArxGripper",
            "left": "ArxGripper"
        }

    @property
    def default_controller_config(self):
        """
        Since this is bimanual robot, returns dict with `'right'`, `'left'` keywords corresponding to their respective
        values

        Returns:
            dict: Dictionary containing arm-specific default controller config names
        """
        return {
            "right": "default_arxlift",
            "left": "default_arxlift",
        }

    @property
    def init_qpos(self):
        return np.array([0.0, 0.3, 0.7, -0.67, 0.0, 0.12,
                         0.0, 0.3, 0.7, -0.67, 0.0, 0.12])

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
        return "bimanual"

    @property
    def _eef_name(self):
        """
        Since this is bimanual robot, returns dict with `'right'`, `'left'` keywords corresponding to their respective
        values

        Returns:
            dict: Dictionary containing arm-specific eef names
        """
        return {"right": "right_hand", "left": "left_hand"}


@register_robot_class("FixedBaseRobot")
class ArxX7sArmsOnly(ManipulatorModel):

    arms = ["right", "left"]

    def __init__(self, idn=0):
        super().__init__(robosuite_model_path_completion("robots/arx_x7s/dual_robot.xml"), idn=idn)

        # Set joint damping
        self.set_joint_attribute(attrib="damping",
                                 values=np.array((0.1, 0.1, 0.1, 0.1, 0.1, 0.01, 0.01,
                                                  0.1, 0.1, 0.1, 0.1, 0.1, 0.01, 0.01)))

    @property
    def default_base(self):
        return "NullMount"

    @property
    def default_gripper(self):
        return {
            "right": "ArxX7Gripper",
            "left": "ArxX7Gripper"
        }

    @property
    def default_controller_config(self):
        """
        Since this is bimanual robot, returns dict with `'right'`, `'left'` keywords corresponding to their respective
        values

        Returns:
            dict: Dictionary containing arm-specific default controller config names
        """
        return {
            "right": "default_arxlift",
            "left": "default_arxlift",
        }

    @property
    def init_qpos(self):
        return np.zeros(14)

    @property
    def base_xpos_offset(self):
        return {
            "bins": (-0.5, -0.1, 0),
            "empty": (-0.6, 0, 0),
            "table": lambda table_length: (-table_length / 2, 0.0, 1.2),
        }

    @property
    def top_offset(self):
        return np.array((0, 0, 1.0))

    @property
    def _horizontal_radius(self):
        return 0.5

    @property
    def arm_type(self):
        return "bimanual"

    @property
    def _eef_name(self):
        """
        Since this is bimanual robot, returns dict with `'right'`, `'left'` keywords corresponding to their respective
        values

        Returns:
            dict: Dictionary containing arm-specific eef names
        """
        return {"right": "right_hand", "left": "left_hand"}