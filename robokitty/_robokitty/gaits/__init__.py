"""
Locomotion Behavior Engine Registry.
"""

from .base import Gait
from .stand import Stand
from .crawl import Crawl

__all__ = [
    "Gait",
    "Stand",
    "Crawl",
]
