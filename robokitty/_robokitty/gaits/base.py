"""
Base interface definition for all quadruped locomotion gaits.
"""

from abc import ABC, abstractmethod


class Gait(ABC):
    """
    Abstract Base Class defining the life-cycle framework for locomotion gaits.
    """

    _registry: dict[str, type["Gait"]] = {}

    def __init_subclass__(cls, alias: str | None = None, **kwargs):
        super().__init_subclass__(**kwargs)
        if alias is not None:
            Gait._registry[alias] = cls

    @classmethod
    def create(cls, name: str, *args, **kwargs) -> "Gait":
        klass = cls._registry.get(name.lower())
        if klass is None:
            raise ValueError(f"Unknown gait: '{name}'. Options: {list(cls._registry)}")
        return klass(*args, **kwargs)

    def __init__(self, period_ticks: int = 40):
        self.period_ticks = period_ticks

    @abstractmethod
    def get_pose(
        self, tick: int, leg_map: dict[str, tuple[int, int, int]]
    ) -> dict[int, list[int]]:
        """
        Calculate register positions for all joints at a given clock tick.

        Parameters
        ----------
        tick : int
            The current clock index of the locomotion loop.
        leg_map : dict
            Mapping of leg keys ("FL", "FR", etc.) to their joint ID triples.

        Returns
        -------
        dict
            A packet payload dictionary mapping servo_id -> [Low Byte, High Byte]
        """
        pass
