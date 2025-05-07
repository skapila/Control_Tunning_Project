from core.flight_mode import FlightMode
from utils.logger import Logger

class AltHoldMode:
    def __init__(self, alt_pid, angle_pid_roll, rate_pid_roll, angle_pid_pitch, rate_pid_pitch, rate_pid_yaw, mixer, sensors, esc):
        self.alt_pid = alt_pid
        self.angle_pid_roll = angle_pid_roll
        self.rate_pid_roll = rate_pid_roll
        self.angle_pid_pitch = angle_pid_pitch
        self.rate_pid_pitch = rate_pid_pitch
        self.rate_pid_yaw = rate_pid_yaw
        self.mixer = mixer
        self.sensors = sensors
        self.esc = esc
        self.target_altitude = None
        self.hover_range_limit=30
       

    def update(self, pilot_input, dt):
        current_altitude = self.sensors.read_alt()

        if self.target_altitude is None:
           self.target_altitude = current_altitude
           Logger.info(f"[ALT_HOLD] Locking target altitude at: {self.target_altitude:.2f} m")

        # adjust target altitude based on throttle PWM
        throttle_pwm = pilot_input.get_throttle_pwm()
        if self.alt_pid.hover_pwm - self.hover_range_limit <= throttle_pwm <= self.alt_pid.hover_pwm + self.hover_range_limit:
           # do nothing, hold
           pass
        elif throttle_pwm > 1520:
             self.target_altitude += 0.3
        elif throttle_pwm < 1480:
             self.target_altitude -= 0.3

        altitude_thrust_pwm = self.alt_pid.compute(self.target_altitude, current_altitude, dt)

        # roll control
        desired_roll = pilot_input.get_desired_roll_angle()
        actual_roll = self.sensors.read_roll()
        desired_rate_roll = self.angle_pid_roll.compute(desired_roll, actual_roll, dt)
        actual_rate_roll = self.sensors.read_roll_rate()
        torque_roll = self.rate_pid_roll.compute(desired_rate_roll, actual_rate_roll, dt)

        # pitch control
        desired_pitch = pilot_input.get_desired_pitch_angle()
        actual_pitch = self.sensors.read_pitch()
        desired_rate_pitch = self.angle_pid_pitch.compute(desired_pitch, actual_pitch, dt)
        actual_rate_pitch = self.sensors.read_pitch_rate()
        torque_pitch = self.rate_pid_pitch.compute(desired_rate_pitch, actual_rate_pitch, dt)

        # yaw control
        desired_rate_yaw = pilot_input.get_desired_yaw_rate()
        actual_rate_yaw = self.sensors.read_yaw_rate()
        torque_yaw = self.rate_pid_yaw.compute(desired_rate_yaw, actual_rate_yaw, dt)

        pwm_outputs = self.mixer.mix(altitude_thrust_pwm, torque_pitch, torque_roll, torque_yaw)
        self.esc.send_pwm(self.sensors, pwm_outputs)
        
        Logger.debug(f"[ALT_HOLD] Alt: {current_altitude:.2f}, Target: {self.target_altitude:.2f}, PWM: {altitude_thrust_pwm:.2f}")

