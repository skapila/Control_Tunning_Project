from controller.angle_pid import AnglePID
from controller.rate_pid import RatePID
from controller.motor_mixer import MotorMixer
from sensors.mavlink_sensor import MavlinkSensor
from input.pilot_input import PilotInput
from output.esc_driver import ESCDriver
from core.stabilize_mode import StabilizeMode
from utils.logger import Logger
import time

if __name__ == "__main__":
    angle_pid = AnglePID(kP=4.5)
    rate_pid = RatePID(kP=0.15, kI=0.01, kD=0.02)
    mixer = MotorMixer()
    sensors = MavlinkSensor('tcp:127.0.0.1:5762')
    esc = ESCDriver()
    pilot_input = PilotInput()

    mode = StabilizeMode(angle_pid, rate_pid, mixer, sensors, esc)

    while True:
        dt = 0.1
        mode.update(pilot_input, dt)
        time.sleep(dt)
