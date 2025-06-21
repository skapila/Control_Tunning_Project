from core.flight_mode import FlightMode
from utils.logger import Logger

class GuidedMode(FlightMode):
    def __init__(self, position_pid_x, position_pid_y,position_pid_z,
                 velocity_pid_x, velocity_pid_y,
                 angle_pid_roll, rate_pid_roll,
                 angle_pid_pitch, rate_pid_pitch, rate_pid_yaw,
                 altitude_pid, mixer, sensors, esc):
                 
        self.position_pid_x = position_pid_x
        self.position_pid_y = position_pid_y
        self.velocity_pid_x = velocity_pid_x
        self.velocity_pid_y = velocity_pid_y
        self.position_pid_z = position_pid_z  # only position pid in z
        self.angle_pid_roll = angle_pid_roll
        self.rate_pid_roll = rate_pid_roll
        self.angle_pid_pitch = angle_pid_pitch
        self.rate_pid_pitch = rate_pid_pitch
        self.rate_pid_yaw = rate_pid_yaw
        self.altitude_pid = altitude_pid
        self.mixer = mixer
        self.sensors = sensors
        self.esc = esc

        self.target_x = None
        self.target_y = None
        self.target_altitude = None  # locked at first entry

    def set_target_offset(self, dx=0.0, dy=0.0, alt=0.0):
        current_lat, current_lon = self.sensors.read_latlon()
        self.target_x = current_lat + dx  # assuming simplified local frame for demo
        self.target_y = current_lon + dy
        self.target_alt = alt
        
    def update(self, pilot_input, dt):
        # Read current state
        current_x, current_y = self.sensors.read_latlon()
        current_alt = self.sensors.read_alt()
        
        if self.target_x is None or self.target_y is None or self.target_alt is None:
            self.set_target_offset(0.0, 0.0, current_alt)
            Logger.info(f"[GUIDED] Locked initial target at: ({self.target_x}, {self.target_y}, {self.target_alt})")

        # --- POSITION CONTROLLER (X-Y) ---
        pos_error_x = self.target_x - current_x
        pos_error_y = self.target_y - current_y

        desired_vx = self.position_pid_x.compute(self.target_x, current_x, dt)
        desired_vy = self.position_pid_y.compute(self.target_y, current_y, dt)

        # Read current velocities (NED frame)
        vx, vy, vz = self.sensors.read_velocity_ned()
        
        # Velocity control for XY
        velocity_cmd_x = self.velocity_pid_x.compute(desired_vx, vx, dt)
        velocity_cmd_y = self.velocity_pid_y.compute(desired_vy, vy, dt)

        # Angle control from velocity control
        desired_pitch = -velocity_cmd_x
        desired_roll = velocity_cmd_y

        # --- ALTITUDE CONTROLLER ---
        vz_cmd = self.position_pid_x.compute(self.target_alt, current_alt, dt)
        current_vz = -vz  # Convert NED downward to positive upward
        altitude_thrust_pwm = self.altitude_pid.compute(vz_cmd, current_vz, dt)

        # ---------- ANGLE PID LOOP ----------
        actual_roll = self.sensors.read_roll()
        desired_rate_roll = self.angle_pid_roll.compute(desired_roll, actual_roll, dt)
        actual_rate_roll = self.sensors.read_roll_rate()
        torque_roll = self.rate_pid_roll.compute(desired_rate_roll, actual_rate_roll, dt)

        actual_pitch = self.sensors.read_pitch()
        desired_rate_pitch = self.angle_pid_pitch.compute(desired_pitch, actual_pitch, dt)
        actual_rate_pitch = self.sensors.read_pitch_rate()
        torque_pitch = self.rate_pid_pitch.compute(desired_rate_pitch, actual_rate_pitch, dt)

        desired_rate_yaw = pilot_input.get_desired_yaw_rate()
        actual_rate_yaw = self.sensors.read_yaw_rate()
        torque_yaw = self.rate_pid_yaw.compute(desired_rate_yaw, actual_rate_yaw, dt)

        pwm_outputs = self.mixer.mix(altitude_thrust_pwm, torque_pitch, torque_roll, torque_yaw)
        self.esc.send_pwm(self.sensors, pwm_outputs)

        Logger.debug(f"[GUIDED] Pos: ({current_x if current_x is not None else 0:.2f}, "
             f"{current_y if current_y is not None else 0:.2f}, "
             f"{current_alt if current_alt is not None else 0:.2f}), "
             f"Target: ({self.target_x if self.target_x is not None else 0:.2f}, "
             f"{self.target_y if self.target_y is not None else 0:.2f}, "
             f"{self.target_alt if self.target_alt is not None else 0:.2f}), "
             f"Vz_cmd: {vz_cmd:.2f}, PWM: {altitude_thrust_pwm:.2f}")
        Logger.debug(f"[GUIDED] Error | dX: {pos_error_x:.2f} m, dY: {pos_error_y:.2f} m, dZ: {self.target_alt - current_alt:.2f} m")
        Logger.debug(f"[GUIDED] PWM: {altitude_thrust_pwm:.2f}")
        


