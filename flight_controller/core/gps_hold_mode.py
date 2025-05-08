from utils.logger import Logger

class GPSHoldMode:
    def __init__(self, position_pid_x, position_pid_y, angle_pid_roll, rate_pid_roll,
                 angle_pid_pitch, rate_pid_pitch, rate_pid_yaw, alt_pid,
                 mixer, sensors, esc):
        self.position_pid_x = position_pid_x
        self.position_pid_y = position_pid_y
        self.angle_pid_roll = angle_pid_roll
        self.rate_pid_roll = rate_pid_roll
        self.angle_pid_pitch = angle_pid_pitch
        self.rate_pid_pitch = rate_pid_pitch
        self.rate_pid_yaw = rate_pid_yaw
        self.alt_pid = alt_pid
        self.mixer = mixer
        self.sensors = sensors
        self.esc = esc

        self.target_position = None  # (lat, lon)
        self.target_altitude = None

    def update(self, pilot_input, dt):
        lat, lon = self.sensors.read_latlon()
        vx, vy, _ = self.sensors.read_velocity_ned()
        current_alt = self.sensors.read_alt()

        if self.target_position is None:
            self.target_position = (lat, lon)
        if self.target_altitude is None:
            self.target_altitude = current_alt

        vx_cmd = self.position_pid_x.compute(self.target_position[0], lat, dt)
        vy_cmd = self.position_pid_y.compute(self.target_position[1], lon, dt)

        desired_pitch = vx_cmd
        desired_roll = -vy_cmd

        actual_roll = self.sensors.read_roll()
        desired_rate_roll = self.angle_pid_roll.compute(desired_roll, actual_roll, dt)
        actual_rate_roll = self.sensors.read_roll_rate()
        torque_roll = self.rate_pid_roll.compute(desired_rate_roll, actual_rate_roll, dt)

        actual_pitch = self.sensors.read_pitch()
        desired_rate_pitch = self.angle_pid_pitch.compute(desired_pitch, actual_pitch, dt)
        actual_rate_pitch = self.sensors.read_pitch_rate()
        torque_pitch = self.rate_pid_pitch.compute(desired_rate_pitch, actual_rate_pitch, dt)

        desired_yaw_rate = pilot_input.get_desired_yaw_rate()
        actual_yaw_rate = self.sensors.read_yaw_rate()
        torque_yaw = self.rate_pid_yaw.compute(desired_yaw_rate, actual_yaw_rate, dt)

        altitude_pwm = self.alt_pid.compute(self.target_altitude, current_alt, dt)

        pwm_outputs = self.mixer.mix(altitude_pwm, torque_pitch, torque_roll, torque_yaw)
        self.esc.send_pwm(self.sensors, pwm_outputs)
