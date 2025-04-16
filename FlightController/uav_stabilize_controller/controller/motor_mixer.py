from utils.logger import Logger

class MotorMixer:
    def __init__(self):
        self.roll_factor=2.5
        self.pitch_factor=2.5
        self.yaw_factor=2.5
      
    def mix(self, throttle, pitch, roll, yaw):
        m1 = int(throttle + self.pitch_factor*pitch + self.roll_factor*roll - self.yaw_factor*yaw)
        m2 = int(throttle + self.pitch_factor*pitch - self.roll_factor*roll + self.yaw_factor*yaw)
        m3 = int(throttle - self.pitch_factor*pitch - self.roll_factor*roll - self.yaw_factor*yaw)
        m4 = int(throttle - self.pitch_factor*pitch + self.roll_factor*roll + self.yaw_factor*yaw)
        pwm_outputs=[self._clamp_pwm(m) for m in [m1, m2, m3, m4]]
        Logger.debug(f"Mixed PWM Outputs: {pwm_outputs}")
        return pwm_outputs

    def _clamp_pwm(self, value, min_pwm=1000, max_pwm=2000):
        return max(min_pwm, min(max_pwm, value))
