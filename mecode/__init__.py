"""Top-level package for mecode."""

__author__ = """Rodrigo Telles"""
__email__ = "rtelles@g.harvard.edu"
__version__ = "0.4.16"

from mecode.main import G, is_str, decode2To3
from mecode.matrix import GMatrix
from mecode.matrix3D import GMatrix3D

__all__ = ["G", "GMatrix", "GMatrix3D", "is_str", "decode2To3"]
