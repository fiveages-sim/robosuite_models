"""
ARX Mobile Base
"""
import numpy as np
from robosuite.models.bases import register_base
from robosuite.models.bases.mobile_base_model import MobileBaseModel

from robosuite_models import robosuite_model_path_completion

@register_base
class ArxLiftBase(MobileBaseModel):
    """
    ARX Mobile Base

    Args:
        idn (int or str): Number or some other unique identification string for this mount instance
    """

    def __init__(self, idn=0):
        super().__init__(robosuite_model_path_completion("bases/arx_lift_base.xml"), idn=idn)

    @property
    def top_offset(self):
        return np.array((0, 0, 0))

    @property
    def horizontal_radius(self):
        return 0.25

@register_base
class ArxX7sBase(MobileBaseModel):
    """
    ARX X7s Mobile Base

    Args:
        idn (int or str): Number or some other unique identification string for this mount instance
    """

    def __init__(self, idn=0):
        super().__init__(robosuite_model_path_completion("bases/arx_x7s_base.xml"), idn=idn)

    @property
    def top_offset(self):
        return np.array((0, 0, 0))

    @property
    def horizontal_radius(self):
        return 0.25