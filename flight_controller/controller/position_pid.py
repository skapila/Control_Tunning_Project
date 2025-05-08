class PositionPID:
    def __init__(self, kP, kI=0.0, kD=0.0):
        self.kP = kP
        self.kI = kI
        self.kD = kD
        self.integral = 0
        self.prev_error = 0

    def compute(self, target, current, dt):
        error = target - current
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        self.prev_error = error
        return self.kP * error + self.kI * self.integral + self.kD * derivative
