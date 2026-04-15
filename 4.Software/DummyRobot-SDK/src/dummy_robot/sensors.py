from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Optional
import time

from .types import SensorFrame


class SensorInterface(ABC):
    """Interface for external sensors such as cameras or custom acquisition modules."""

    name: str

    @abstractmethod
    def read(self) -> Optional[SensorFrame]:
        """Return the latest sensor frame, or None if unavailable."""


class CallbackSensor(SensorInterface):
    """Wrap a callback so experimental sensors can be mounted quickly."""

    def __init__(self, name, callback):
        self.name = name
        self._callback = callback

    def read(self) -> Optional[SensorFrame]:
        data = self._callback()
        if data is None:
            return None
        return SensorFrame(name=self.name, timestamp=time.time(), data=data)
