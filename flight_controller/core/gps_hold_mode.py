import math
from utils.logger import Logger

class GPSHoldMode:
    def __init__(self, position_pid_x, position_pid_y, velocity_pid_x, velocity_pid_y,
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

        self.target_position = None
        self.target_altitude = None
        self.locked_position = None  # GPS origin
        self.dx_local = 0.0
        self.dy_local = 0.0

    def activate(self):
        lat, lon = self.sensors.read_latlon()
        self.locked_position = (lat, lon)
        self.target_position = (lat, lon)
        self.target_altitude = self.sensors.read_alt()
        self.dx_local = 0.0
        self.dy_local = 0.0
        Logger.info(f"[GPS_HOLD] Locked position: {self.target_position}, Alt: {self.target_altitude:.2f}")

    def gps_to_local(self, lat1, lon1, lat0, lon0):
        scale_lat = 111320
        scale_lon = 111320 * math.cos(math.radians(lat0))
        dx = (lat1 - lat0) * scale_lat
        dy = (lon1 - lon0) * scale_lon
        return dx, dy

    def local_to_gps(self, dx, dy, lat0, lon0):
        scale_lat = 1 / 111320
        scale_lon = 1 / (111320 * math.cos(math.radians(lat0)))
        dlat = dx * scale_lat
        dlon = dy * scale_lon
        return lat0 + dlat, lon0 + dlon

    def update(self, pilot_input, dt):
    # --- Safety check
        if dt <= 0.0 or dt > 1.0:
            Logger.warning("[GPS_HOLD] Skipping update due to bad dt")
            return
 
        if self.target_altitude is None:
           self.activate()
    # --- Read state
       # lat, lon = self.sensors.read_latlon()
        vx, vy, _ = self.sensors.read_velocity_ned()
        current_alt = self.sensors.read_alt()

    # --- Lock target on first call
       # if self.locked_position is None or self.target_position is None:
          # self.activate()

    # --- Joystick override to shift target
        roll_pwm = pilot_input.get_roll_pwm()
        pitch_pwm = pilot_input.get_pitch_pwm()
        
        vel_scale = 0.005  # max ±2.5 m/s

        if roll_pwm > 1520 or roll_pwm < 1480:
           vx_des = (roll_pwm - 1500) * vel_scale
        else:
           vx_des = 0.0

        if pitch_pwm > 1520 or pitch_pwm < 1480:
           vy_des = (pitch_pwm - 1500) * vel_scale
        else:
            vy_des = 0.0
        
        #if abs(dx_offset) > 0.5 or abs(dy_offset) > 0.5:
         #  self.dx_local += dx_offset
          # self.dy_local += dy_offset
          # self.dx_local = max(-10.0, min(10.0, self.dx_local))
          # self.dy_local = max(-10.0, min(10.0, self.dy_local))
           #self.target_position = self.local_to_gps(self.dx_local, self.dy_local, *self.locked_position)

    # --- Position control
       # dx, dy = self.gps_to_local(lat, lon, *self.target_position)
       # vx_des = self.position_pid_x.compute(0.0, dx, dt)
        #vy_des = self.position_pid_y.compute(0.0, dy, dt)
#
    # --- Velocity control
        desired_roll = self.velocity_pid_x.compute(vx_des, vx, dt)*10
        desired_pitch = self.velocity_pid_y.compute(vy_des, vy, dt)*10
   
    # --- Sensor  read
        actual_roll = self.sensors.read_roll()
        actual_pitch = self.sensors.read_pitch()
        actual_rate_roll = self.sensors.read_roll_rate()
        actual_rate_pitch = self.sensors.read_pitch_rate()
        actual_yaw_rate = self.sensors.read_yaw_rate()

        desired_rate_roll = self.angle_pid_roll.compute(desired_roll, actual_roll, dt)
        torque_roll = self.rate_pid_roll.compute(desired_rate_roll, actual_rate_roll, dt)

        desired_rate_pitch = self.angle_pid_pitch.compute(desired_pitch, actual_pitch, dt)
        torque_pitch = self.rate_pid_pitch.compute(desired_rate_pitch, actual_rate_pitch, dt)

        desired_yaw_rate = pilot_input.get_desired_yaw_rate()
        torque_yaw = self.rate_pid_yaw.compute(desired_yaw_rate, actual_yaw_rate, dt)

    # --- Altitude control
        altitude_pwm = int(self.alt_pid.compute(self.target_altitude, current_alt, dt))

    # --- Mix and send
        pwm_outputs = self.mixer.mix(pilot_input.get_throttle_pwm(), torque_pitch, torque_roll, torque_yaw)
        self.esc.send_pwm(self.sensors, pwm_outputs)

    # --- Debug
        Logger.debug(f"[GPS_HOLD] vx_actual: {vx:.2f}, vy_actual: {vy:.2f}")
        Logger.debug(f"[GPS_HOLD] vx_des: {vx_des:.2f}, vy_des: {vy_des:.2f}")
        Logger.debug(f"[GPS_HOLD] RollCmd: {desired_roll:.2f}, PitchCmd: {desired_pitch:.2f}")
        Logger.debug(f"[GPS_HOLD] Alt: {current_alt:.2f}, TargetAlt: {self.target_altitude:.2f}, ThrottlePWM: {pilot_input.get_throttle_pwm()}")

