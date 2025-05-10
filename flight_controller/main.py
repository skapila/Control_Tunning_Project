from controller.altitude_pid import AltitudePID
from controller.angle_pid import AnglePID
from controller.rate_pid import RatePID
from controller.motor_mixer import MotorMixer
from controller.position_pid import PositionPID
from controller.velocity_pid import VelocityPID
from sensors.mavlink_sensor import MavlinkSensor
from input.pilot_input import PilotInput
from output.esc_driver import ESCDriver
from core.stabilize_mode import StabilizeMode
from core.alt_hold_mode import AltHoldMode
from core.gps_hold_mode import GPSHoldMode
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
    alt_pid = AltitudePID(kP=1.5, kI=0.003, kD=0.5, hover_pwm=1630)
    position_pid_x = PositionPID(kP=60, kI=0.02, kD=0)
    position_pid_y = PositionPID(kP=60, kI=0.02, kD=0)
    velocity_pid_x = VelocityPID(kP=0.8, kI=0.005, kD=0.2)
    velocity_pid_y = VelocityPID(kP=1.2, kI=0.01, kD=0.5)



    # Setup all components
    mixer = MotorMixer()
    sensors = MavlinkSensor('tcp:127.0.0.1:5762')
    esc = ESCDriver()
    pilot_input = PilotInput()

    # Modes
    stabilize_mode = StabilizeMode(angle_pid_roll, rate_pid_roll, angle_pid_pitch, rate_pid_pitch, rate_pid_yaw, mixer, sensors, esc)
    alt_hold_mode = AltHoldMode(alt_pid, angle_pid_roll, rate_pid_roll, angle_pid_pitch, rate_pid_pitch, rate_pid_yaw, mixer, sensors, esc)
    gps_hold_mode = GPSHoldMode(position_pid_x, position_pid_y,velocity_pid_x, velocity_pid_y,
                                angle_pid_roll, rate_pid_roll,
                                angle_pid_pitch, rate_pid_pitch, rate_pid_yaw,
                                alt_pid, mixer, sensors, esc)

    # Mode manager
    mode_manager = ModeManager()
    current = "STABILIZE"

    # Set default mode
    mode_manager.switch_mode(stabilize_mode)
    Logger.info("Starting flight control loop... Press Ctrl+C to stop.")

    while True:
        dt = 0.1

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


        # Update the current mode
        mode_manager.update(pilot_input, dt)
        time.sleep(dt)
