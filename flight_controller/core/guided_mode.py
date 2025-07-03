from core.flight_mode import FlightMode
from utils.logger import Logger
import math

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
        self.target_alt = None  # locked at first entry
        self.home_lat = None
        self.home_lon = None
        self.home_alt = None
        self.ground_speed = 4
        self.vertical_speed = 6
        
        self.takeoff_active = False
        self.landing_active = False
        self.last_set_alt = 150  # this matches your `set_target_offset` alt
        
        self.takeoff_enabled = False  
        self.takeoff_complete = False
        self.takeoff_altitude = 10.0  # meters above home_alt


       
        
    def smooth_velocity(self,distance_to_target, max_speed=4, slowdown_radius=10.0):
        if distance_to_target >= slowdown_radius:
           return max_speed
        elif distance_to_target <= 0.1:
           return 0.0  # considered arrived
        else:
          # Smooth scaling using cosine interpolation
           scale = 0.5 * (1 - math.cos(math.pi * distance_to_target / slowdown_radius))
           return scale * max_speed
    
        
    def geodetic_to_ned(self,lat1, lon1, lat2, lon2):
        # Constants
        R_EARTH = 6378137.0  # Earth's radius in meters (WGS-84)
    
        # Convert degrees to radians
        lat1_rad = math.radians(lat1)
        lon1_rad = math.radians(lon1)
        lat2_rad = math.radians(lat2)
        lon2_rad = math.radians(lon2)
    
        # Differences in radians
        dLat = lat2_rad - lat1_rad
        dLon = lon2_rad - lon1_rad

        # NED displacements
        dNorth = dLat * R_EARTH
        dEast = dLon * R_EARTH * math.cos(lat1_rad)

        return dNorth, dEast    
        
    def ned_to_geodetic(self,lat1, lon1, dNorth, dEast):
       # Constants
       R_EARTH = 6378137.0  # Earth's radius in meters (WGS-84)

       # Convert origin latitude to radians
       lat1_rad = math.radians(lat1)

       # Calculate new latitude
       dLat = dNorth / R_EARTH
       lat2_rad = lat1_rad + dLat

       # Calculate new longitude
       dLon = dEast / (R_EARTH * math.cos(lat1_rad))
       lon2_rad = math.radians(lon1) + dLon

       # Convert back to degrees
       lat2 = math.degrees(lat2_rad)
       lon2 = math.degrees(lon2_rad)

       return lat2, lon2    

    def set_target_offset(self, dx=0.0, dy=0.0, alt=None):
        current_lat, current_lon = self.sensors.read_latlon()
        current_alt = self.sensors.read_alt()
        self.target_lat,self.target_lon = self.ned_to_geodetic(current_lat,current_lon,dx,dy)
        self.target_x,self.target_y = self.geodetic_to_ned(self.home_lat, self.home_lon,self.target_lat, self.target_lon)
       
        if alt is not None:
           self.target_alt = self.home_alt - alt           #####with ned reference
        
    def unit_vector(self,vx, vy):
        magnitude = math.sqrt(vx**2 + vy**2)
        if magnitude == 0:
           magnitude = 0.1
        
        return (vx / magnitude, vy / magnitude)
        
     
        
    def update(self, pilot_input, dt):
        # Read current state
        current_lat, current_lon = self.sensors.read_latlon()
        current_alt = self.sensors.read_alt()
        
        if self.target_x is None or self.target_y is None or self.target_alt is None:
            self.set_target_offset(0.0, 0.0, current_alt)
            Logger.info(f"[GUIDED] Locked initial target at: ({self.target_x}, {self.target_y}, {self.target_alt})")
            
        # --- Handle Takeoff / Landing Commands ---
        if pilot_input.takeoff_pressed:
           if not self.takeoff_active:
              Logger.info("[GUIDED] Takeoff initiated")
              self.takeoff_active = True
              self.landing_active = False
              self.target_alt = self.home_alt - self.last_set_alt  # NED reference

        elif pilot_input.land_pressed:
           if not self.landing_active:
              Logger.info("[GUIDED] Landing initiated")
              self.landing_active = True
              self.takeoff_active = False
              self.target_alt = self.home_alt  # ground level

        # --- POSITION CONTROLLER (X-Y) ---
        current_x,current_y = self.geodetic_to_ned(self.home_lat, self.home_lon,current_lat, current_lon)
        pos_error_x = self.target_x - current_x
        pos_error_y = self.target_y - current_y

        desired_vx = self.position_pid_x.compute(self.target_x, current_x, dt)
        desired_vy = self.position_pid_y.compute(self.target_y, current_y, dt)
        
        # Read current velocities (NED frame)
        vx, vy, vz = self.sensors.read_velocity_ned()
        uv_x, uv_y= self.unit_vector(desired_vx, desired_vy)
        yaw = self.sensors.read_yaw()  # In radians
        
     
        # Velocity control for XY
        distance = math.sqrt(pos_error_x**2 + pos_error_y**2)
        speed = self.smooth_velocity(distance,max_speed = self.ground_speed )
        velocity_cmd_x = self.velocity_pid_x.compute(uv_x*speed, vx, dt)
        velocity_cmd_y = self.velocity_pid_y.compute(uv_y*speed, vy, dt)
        
         # --- Convert NED velocity to Body Frame
        vx_body = math.cos(yaw) * velocity_cmd_x + math.sin(yaw) * velocity_cmd_y    # forward/backward
        vy_body = -math.sin(yaw) * velocity_cmd_x + math.cos(yaw) * velocity_cmd_y   # left/right

        # Angle control from velocity control
        desired_pitch = -vx_body    #### positive x is north
        desired_roll = vy_body 

        # --- ALTITUDE CONTROLLER ---
        
        diff_alt = self.target_alt - current_alt
        vz_speed = self.smooth_velocity(abs(diff_alt),max_speed=self.vertical_speed)
        vz_cmd = self.position_pid_x.compute(self.target_alt, current_alt, dt)
        vz_speed = vz_speed*math.copysign(1,vz_cmd)
        current_vz = -vz  # Convert NED downward to positive upward
        #altitude_thrust_pwm = self.altitude_pid.compute(vz_speed, current_vz, dt)
        
        altitude_thrust_pwm = self.altitude_pid.compute(vz_speed, current_vz, dt)

        # Takeoff logic
        if self.takeoff_enabled and not self.takeoff_complete:
            target_takeoff_alt = self.home_alt - self.takeoff_altitude
            if current_alt >= target_takeoff_alt - 0.5:
               self.takeoff_complete = True
               self.takeoff_enabled = False
               Logger.info("[GUIDED] Takeoff complete.")
            else:
        # Apply override thrust to lift-off
               if altitude_thrust_pwm < 1650:
                  altitude_thrust_pwm = 1650


        # ---------- ANGLE PID LOOP ----------
        actual_roll = self.sensors.read_roll()
        desired_rate_roll = self.angle_pid_roll.compute(desired_roll, actual_roll, dt)
        actual_rate_roll = self.sensors.read_roll_rate()
        torque_roll = self.rate_pid_roll.compute(desired_rate_roll, actual_rate_roll, dt)

        actual_pitch = self.sensors.read_pitch()
        desired_rate_pitch = self.angle_pid_pitch.compute(desired_pitch, actual_pitch, dt)
        actual_rate_pitch = self.sensors.read_pitch_rate()
        torque_pitch = self.rate_pid_pitch.compute(desired_rate_pitch, actual_rate_pitch, dt)

        desired_rate_yaw = 0.0
        actual_rate_yaw = self.sensors.read_yaw_rate()
        torque_yaw = self.rate_pid_yaw.compute(desired_rate_yaw, actual_rate_yaw, dt)

        pwm_outputs = self.mixer.mix(altitude_thrust_pwm, torque_pitch, torque_roll, torque_yaw)
        self.esc.send_pwm(self.sensors, pwm_outputs)

        Logger.debug("[GUIDED] Pos: ("+str(current_x)+","+str(current_y)+","+str(current_alt)+")")
        Logger.debug("[GUIDED] Target: ("+str(self.target_x)+","+str(self.target_y)+","+str(self.target_alt)+")")
        Logger.debug("[GUIDED] Target: ("+str(vx)+","+str(vy)+","+str(vz)+")")
        Logger.debug(f"Vz_cmd: {vz_cmd:.2f}, PWM: {altitude_thrust_pwm:.2f}")
        Logger.debug(f"[GUIDED] Error | dX: {pos_error_x:.2f} m, dY: {pos_error_y:.2f} m, dZ: {diff_alt} m")
        Logger.debug(f"[GUIDED] PWM: {altitude_thrust_pwm:.2f}")
        


