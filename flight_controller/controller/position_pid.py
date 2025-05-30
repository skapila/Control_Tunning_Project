class PositionPID:
    def __init__(self, kP=0.0, kI=0.0, kD=0.0):
        self.kP = kP
        self.kI = kI
        self.kD = kD
        self.integral = 0.0
        self.last_error = 0.0

    def compute(self, target, current, dt):
        error = target - current
        self.integral += error * dt
        derivative = (error - self.last_error) / dt if dt > 0 else 0.0
        self.last_error = error

        return self.kP * error + self.kI * self.integral + self.kD * derivative
