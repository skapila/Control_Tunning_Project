import math
from utils.logger import Logger

class GuidedMode:
    def __init__(self, position_pid_x, position_pid_y,
                 velocity_pid_x, velocity_pid_y,
                 angle_pid_roll, rate_pid_roll, angle_pid_pitch, rate_pid_pitch, rate_pid_yaw,
                 alt_pid, mixer, sensors, esc):

        self.position_pid_x = position_pid_x
        self.position_pid_y = position_pid_y
        self.velocity_pid_x = velocity_pid_x
        self.velocity_pid_y = velocity_pid_y
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
        self.target_altitude = None  # m
        self.locked_position = None  # origin for NED

    def activate(self):
        current_lat, current_lon = self.sensors.read_latlon()
        current_alt = self.sensors.read_alt()
        self.locked_position = (current_lat, current_lon)
        if self.target_position is None:
            self.target_position = (current_lat, current_lon)
        if self.target_altitude is None:
            self.target_altitude = current_alt
        Logger.info(f"[GUIDED] Activated with target: {self.target_position} Alt: {self.target_altitude:.2f}")

    def set_target_position(self, lat, lon, alt):
        self.target_position = (lat, lon)
        self.target_altitude = alt
        Logger.info(f"[GUIDED] New target position: {self.target_position} Alt: {alt:.2f}")

    def gps_to_local(self, lat1, lon1, lat0, lon0):
        scale_lat = 111320
        scale_lon = 111320 * math.cos(math.radians(lat0))
        dx = (lat1 - lat0) * scale_lat
        dy = (lon1 - lon0) * scale_lon
        return dx, dy

    def update(self, pilot_input, dt):
        if dt <= 0.0 or dt > 1.0:
            Logger.warning("[GUIDED] Skipping update due to bad dt")
            return

        # Current readings
        current_lat, current_lon = self.sensors.read_latlon()
        current_alt = self.sensors.read_alt()
        yaw = self.sensors.read_yaw()
        vx_ned, vy_ned, _ = self.sensors.read_velocity_ned()

        # --- Convert target GPS to local offset
        dx, dy = self.gps_to_local(self.target_position[0], self.target_position[1],
                                   self.locked_position[0], self.locked_position[1])
        cx, cy = self.gps_to_local(current_lat, current_lon,
                                   self.locked_position[0], self.locked_position[1])
        dx_err = dx - cx
        dy_err = dy - cy

        # --- Position to Velocity
        vx_des = self.position_pid_x.compute(dx_err, 0.0, dt)
        vy_des = self.position_pid_y.compute(dy_err, 0.0, dt)

        # --- Convert velocity to body frame
        vx_body = math.cos(yaw) * vx_ned + math.sin(yaw) * vy_ned
        vy_body = -math.sin(yaw) * vx_ned + math.cos(yaw) * vy_ned

        # --- Velocity to Attitude
        desired_roll = self.velocity_pid_x.compute(vy_des, vy_body, dt)
        desired_pitch = -self.velocity_pid_y.compute(vx_des, vx_body, dt)

        # --- Attitude and Rate PID
        actual_roll = self.sensors.read_roll()
        actual_pitch = self.sensors.read_pitch()
        actual_rate_roll = self.sensors.read_roll_rate()
        actual_rate_pitch = self.sensors.read_pitch_rate()
        actual_yaw_rate = self.sensors.read_yaw_rate()

        desired_rate_roll = self.angle_pid_roll.compute(desired_roll, actual_roll, dt)
        desired_rate_pitch = self.angle_pid_pitch.compute(desired_pitch, actual_pitch, dt)

        torque_roll = self.rate_pid_roll.compute(desired_rate_roll, actual_rate_roll, dt)
        torque_pitch = self.rate_pid_pitch.compute(desired_rate_pitch, actual_rate_pitch, dt)

        # Yaw deadband filter
        yaw_pwm = pilot_input.get_yaw_pwm()
        if 1480 <= yaw_pwm <= 1520:
            desired_yaw_rate = 0.0
        else:
            desired_yaw_rate = pilot_input.get_desired_yaw_rate()
        torque_yaw = self.rate_pid_yaw.compute(desired_yaw_rate, actual_yaw_rate, dt)

        # --- Altitude hold
        throttle_pwm = int(self.alt_pid.compute(self.target_altitude, current_alt, dt))

        # --- Mix and send
        pwm_outputs = self.mixer.mix(throttle_pwm, torque_pitch, torque_roll, torque_yaw)
        self.esc.send_pwm(self.sensors, pwm_outputs)

        Logger.debug(f"[GUIDED] dx_err={dx_err:.2f}, dy_err={dy_err:.2f}")
        Logger.debug(f"[GUIDED] vx_des={vx_des:.2f}, vy_des={vy_des:.2f}")

