from interfaces.pid_interface import PIDInterface

class AnglePID(PIDInterface):
    def __init__(self, kP):
        self.kP = kP

    def compute(self, target, current, dt):
        error = target - current
        return self.kP * error
