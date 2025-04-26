from abc import ABC, abstractmethod

class FlightMode(ABC):
    @abstractmethod
    def update(self, pilot_input, dt):
        pass
