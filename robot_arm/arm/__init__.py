"""Robotic arm control package (GRBL + CNC Shield V3)."""

from .config import ArmConfig
from .grbl import GrblController

__all__ = ["ArmConfig", "GrblController", "create_app"]
__version__ = "1.0.0"


def create_app():
    from .server import create_app as _c
    return _c()
