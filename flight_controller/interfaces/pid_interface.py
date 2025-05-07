from abc import ABC, abstractmethod

class PIDInterface(ABC):
    @abstractmethod
    def compute(self, target: float, current: float, dt: float) -> float:
        pass
