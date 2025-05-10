class AltitudePID:
    def __init__(self, kP, kI, kD, hover_pwm=1580):
        self.kP = kP
        self.kI = kI
        self.kD = kD
        self.hover_pwm = hover_pwm
        self.integral = 0
        self.prev_error = 0

    def compute(self, target, current, dt):
        error = target - current
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        self.prev_error = error

        output = self.kP * error + self.kI * self.integral + self.kD * derivative
        pwm = self.hover_pwm + output
        return max(1300, min(1750, pwm))  # tighter clamp instead of 1000–2000
        
        Logger.debug(f"[ALT_PID] error={error:.2f}, pwm={pwm:.2f}")


