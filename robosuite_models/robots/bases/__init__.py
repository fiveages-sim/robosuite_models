from robosuite.models.bases.mount_model import MountModel
from robosuite.models.bases.mobile_base_model import MobileBaseModel
from robosuite.models.bases.leg_base_model import LegBaseModel

from .aloha_mount import AlohaMount
from .b1_base import B1, B1Floating
from .go2_base import Go2, Go2Floating
from .arx_base import ArxLiftBase, ArxX7sBase

BASE_MAPPING = {
    "AlohaMount": AlohaMount,
    "B1": B1,
    "B1Floating": B1Floating,
    "Go2": Go2,
    "Go2Floating": Go2Floating,
    "ArxLiftBase": ArxLiftBase,
    "ArxX7sBase": ArxX7sBase,
}

ALL_BASES = BASE_MAPPING.keys()
