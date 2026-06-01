from controller.altitude_pid import AltitudePID
from controller.angle_pid import AnglePID
from controller.rate_pid import RatePID
from controller.motor_mixer import MotorMixer
from controller.velocity_pid import VelocityPID
from controller.position_pid import PositionPID
from sensors.mavlink_sensor import MavlinkSensor
from input.sm600_input import SM600Input as PilotInput
from output.esc_driver import ESCDriver
from core.stabilize_mode import StabilizeMode
from core.alt_hold_mode import AltHoldMode
from core.gps_hold_mode import GPSHoldMode
from core.guided_mode import GuidedMode
from core.mode_manager import ModeManager
from utils.logger import Logger
import config.pid_config as pid
import time

if __name__ == "__main__":

    # Controllers
    angle_pid_roll = AnglePID(kP=pid.RLL_ANGLE_KP)
    rate_pid_roll = RatePID(kP=pid.RLL_RATE_KP, kI=pid.RLL_RATE_KI, kD=pid.RLL_RATE_KD)
    angle_pid_pitch = AnglePID(kP=pid.PIT_ANGLE_KP)
    rate_pid_pitch = RatePID(kP=pid.PIT_RATE_KP, kI=pid.PIT_RATE_KI, kD=pid.PIT_RATE_KD)
    rate_pid_yaw = RatePID(kP=pid.YAW_RATE_KP, kI=pid.YAW_RATE_KI, kD=pid.YAW_RATE_KD)
    
    # Altitude controller (works on vertical velocity error)
    alt_pid = AltitudePID(kP=15, kI=3.5, kD=2)
    
    # XY Velocity controllers
    velocity_pid_x = VelocityPID(kP=15, kI=3.5, kD=2)
    velocity_pid_y = VelocityPID(kP=15, kI=3.5, kD=2)
    
    # XY Position controllers
    position_pid_x = PositionPID(kP=0.05, kI=0.0, kD=0.0)
    position_pid_y = PositionPID(kP=0.05, kI=0.0, kD=0.0)
    position_pid_z = PositionPID(kP=0.05, kI=0.0, kD=0.00)

    # Setup all components
    mixer = MotorMixer()
    sensors = MavlinkSensor('tcp:127.0.0.1:5762')
    esc = ESCDriver()
    pilot_input = PilotInput()

    # Flight Modes
    stabilize_mode = StabilizeMode(angle_pid_roll, rate_pid_roll, angle_pid_pitch, rate_pid_pitch, rate_pid_yaw, mixer, sensors, esc)
    alt_hold_mode = AltHoldMode(alt_pid, angle_pid_roll, rate_pid_roll, angle_pid_pitch, rate_pid_pitch, rate_pid_yaw, mixer, sensors, esc)
    gps_hold_mode = GPSHoldMode(velocity_pid_x, velocity_pid_y,
                                angle_pid_roll, rate_pid_roll,
                                angle_pid_pitch, rate_pid_pitch, rate_pid_yaw,
                                alt_pid, mixer, sensors, esc)
    guided_mode = GuidedMode(position_pid_x, position_pid_y,position_pid_z,
                         velocity_pid_x, velocity_pid_y,
                         angle_pid_roll, rate_pid_roll,
                         angle_pid_pitch, rate_pid_pitch, rate_pid_yaw,
                         alt_pid, mixer, sensors, esc)
    #gps_home_lock 
    gps_home_lock = 0

    # Mode manager
    mode_manager = ModeManager()
    current = "STABILIZE"

    # Set default mode
    mode_manager.switch_mode(stabilize_mode)
    Logger.info("Starting flight control loop... Press Ctrl+C to stop.")
    
    # Target offset deltas for guided mode movement (in meters)
    TARGET_DELTA = 30.0
    dx, dy = 0.0, 0.0
    guided_active = False
    
    while True:
        dt = 0.1
        if sensors.gps_home_flag == 1 and gps_home_lock ==0 :
           guided_mode.home_lat, guided_mode.home_lon = sensors.read_latlon()  ###one time read gps
           guided_mode.home_alt = sensors.read_alt()
           gps_home_lock = 1
        
           
        # switch mode based on pilot input
        mode_signal = pilot_input.mode_switch
        if mode_signal == "ALT_HOLD" and current != "ALT_HOLD":
            mode_manager.switch_mode(alt_hold_mode)
            current = "ALT_HOLD"
            Logger.info(f"Switched to {current} mode")

        elif mode_signal == "GPS_HOLD" and current != "GPS_HOLD":
            mode_manager.switch_mode(gps_hold_mode)
            current = "GPS_HOLD"
            Logger.info(f"Switched to {current} mode")

        elif mode_signal == "STABILIZE" and current != "STABILIZE":
            mode_manager.switch_mode(stabilize_mode)
            current = "STABILIZE"
            
            Logger.info(f"Switched to {current} mode")
            
        elif mode_signal == "GUIDED" and current != "GUIDED":
            mode_manager.switch_mode(guided_mode)
            # Set initial target
            current_lat, current_lon = sensors.read_latlon()
            current_alt = sensors.read_alt()
    
          #  guided_mode.set_target_offset(dx=0.0, dy=0.0, alt= -150) 
            
            current = "GUIDED"
            guided_active = True
            Logger.info(f"Switched to {current} mode")
            
          # Handle Guided Mode Target Movement with D-Pad

        if current == "GUIDED" and guided_active:   
        
        # L1 = takeoff → only now set the initial target
           if pilot_input.takeoff_pressed and not guided_mode.takeoff_enabled:

              guided_mode.takeoff_enabled = True
              guided_mode.takeoff_complete = False
              guided_mode.set_target_offset(dx=0, dy=0, alt=-guided_mode.takeoff_altitude)  # Use POSITIVE 150 if target_alt = home_alt - alt
              Logger.info("L1 pressed: Takeoff initiated.")

       # L2 = land → set target_alt to home
           if pilot_input.land_pressed and guided_mode.takeoff_enabled:
              guided_mode.set_target_offset(dx=0.0, dy=0.0, alt=0.0)
              guided_mode.takeoff_enabled = False
              Logger.info("L2 pressed: Landing initiated.")
              
         # --- D-Pad Movement (only after takeoff enabled) ---    
           if guided_mode.takeoff_complete: 
              if pilot_input.dpad_up:
                 dx += TARGET_DELTA
                 dy = 0
                 guided_mode.set_target_offset(dx=dx, dy=dy)
              if pilot_input.dpad_down:
                 dx -= TARGET_DELTA
                 dy = 0
                 guided_mode.set_target_offset(dx=dx, dy=dy)
              if pilot_input.dpad_right:
                 dy += TARGET_DELTA
                 dx = 0
                 guided_mode.set_target_offset(dx=dx, dy=dy)
              if pilot_input.dpad_left:
                 dy -= TARGET_DELTA
                 dx = 0
                 guided_mode.set_target_offset(dx=dx, dy=dy) 
                
           

            

        # Update the current mode
        mode_manager.update(pilot_input, dt)
        time.sleep(dt)
