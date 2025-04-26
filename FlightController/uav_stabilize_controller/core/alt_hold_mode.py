from core.flight_mode import FlightMode
from utils.logger import Logger

class AltHoldMode(FlightMode):

    def __init__(self, altitude_pid, angle_pid_roll, rate_pid_roll, angle_pid_pitch, rate_pid_pitch, rate_pid_yaw, mixer, sensors, esc):
        
        self.altitude_pid = altitude_pid

        self.angle_pid_roll = angle_pid_roll
        self.rate_pid_roll = rate_pid_roll
        
        self.angle_pid_pitch = angle_pid_pitch
        self.rate_pid_pitch = rate_pid_pitch

        self.rate_pid_yaw = rate_pid_yaw
    
        self.sensors = sensors
        self.mixer = mixer
        self.esc = esc
        self.target_altitude = 1.5  # meters

 
    def update(self, pilot_input, dt):
        throttle_pwm = pilot_input.get_throttle_pwm()
        current_alt = self.sensors.read_altitude()

        # Define throttle band and climb rate
        center_min = 1480
        center_max = 1520
        climb_speed_per_pwm = 0.01  # meters per PWM unit per second

        if center_min <= throttle_pwm <= center_max:
          # Stick centered: hold altitude
          self.target_altitude = current_alt
        else:
          # Adjust target altitude smoothly based on how far from center
          pwm_delta = throttle_pwm - 1500
          self.target_altitude += pwm_delta * climb_speed_per_pwm * dt

        # Clamp altitude to safe bounds
        self.target_altitude = max(0.2, min(10.0, self.target_altitude))

        # PID throttle control
        torque_throttle_command = self.altitude_pid.compute(self.target_altitude, current_alt, dt)

        #Roll signal
        desired_roll = pilot_input.get_desired_roll_angle()
        actual_roll = self.sensors.read_roll()
        desired_rate = self.angle_pid_roll.compute(desired_roll, actual_roll, dt)

        actual_rate = self.sensors.read_roll_rate()
        torque_roll_command = self.rate_pid_roll.compute(desired_rate, actual_rate, dt)


        #Pitch signal
        desired_pitch = pilot_input.get_desired_pitch_angle()
        actual_pitch = self.sensors.read_pitch()
        desired_rate = self.angle_pid_pitch.compute(desired_pitch, actual_pitch, dt)

        actual_rate = self.sensors.read_pitch_rate()
        torque_pitch_command = self.rate_pid_pitch.compute(desired_rate, actual_rate, dt)


        #Yaw signal
        desired_rate= pilot_input.get_desired_yaw_rate()
        actual_rate = self.sensors.read_yaw_rate()
        torque_yaw_command = self.rate_pid_yaw.compute(desired_rate, actual_rate, dt)


        esc_pwm_outputs = self.mixer.mix(torque_throttle_command, torque_pitch_command, torque_roll_command, torque_yaw_command)
        Logger.info(f"[ALT_HOLD] Alt: {current_alt:.2f} → PWM: {pwm_outputs}")
        self.esc.send_pwm(pwm_outputs)
