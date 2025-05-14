class VelocityPID:
    def __init__(self, kP, kI, kD):
        self.kP = kP
        self.kI = kI
        self.kD = kD
        self.integral = 0
        self.prev_error = 0

    def compute(self, target_vel, actual_vel, dt):
        error = target_vel - actual_vel
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        self.prev_error = error

        output = self.kP * error + self.kI * self.integral + self.kD * derivative
        return output  # Clamp desired angle in degrees

