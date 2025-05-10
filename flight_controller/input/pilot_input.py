from utils.logger import Logger
import threading
import time
import pygame

class PilotInput:
    def __init__(self):
        self.roll_pwm = 1500
        self.pitch_pwm = 1500
        self.yaw_pwm = 1500
        self.throttle_pwm = 1500

        self.max_angle = 45       # degrees
        self.max_yaw_rate = 100   # deg/sec
        self.min_pwm = 1000
        self.max_pwm = 2000

        self.gps_hold = "GPS_HOLD"
        self.alt_hold = "ALT_HOLD"
        self.stabilize = "STABILIZE"
        self.mode_switch = self.stabilize  # default mode

        self.running = True
        self.thread = threading.Thread(target=self._listen_to_rc, daemon=True)
        self.thread.start()

    def _normalize(self, pwm):
        return (pwm - 1500) / 500.0

    def _normalize_throttle(self, pwm):
        return (pwm - self.min_pwm) / (self.max_pwm - self.min_pwm)

    def get_desired_roll_angle(self):
        return self._normalize(self.roll_pwm) * self.max_angle

    def get_desired_pitch_angle(self):
        return self._normalize(self.pitch_pwm) * self.max_angle

    def get_desired_yaw_rate(self):
        return self._normalize(self.yaw_pwm) * self.max_yaw_rate

    def get_throttle_pwm(self):
        return self.throttle_pwm

    #  Add these to support pilot override in GPS Hold
    def get_roll_pwm(self):
        return self.roll_pwm

    def get_pitch_pwm(self):
        return self.pitch_pwm

    def get_yaw_pwm(self):
        return self.yaw_pwm

    def _scale_joystick(self, value, min_out=1000, max_out=2000):
        return int((value + 1) / 2 * (max_out - min_out) + min_out)

    def _listen_to_rc(self):
        pygame.init()
        pygame.joystick.init()

        if pygame.joystick.get_count() == 0:
            Logger.info("No joystick found!")
            exit()

        joystick = pygame.joystick.Joystick(0)
        joystick.init()
        Logger.info(f"Using joystick: {joystick.get_name()}")

        while self.running:
            pygame.event.pump()

            # Mode switching
            if joystick.get_button(0):
                self.mode_switch = self.gps_hold
            elif joystick.get_button(3):
                self.mode_switch = self.alt_hold
            elif joystick.get_button(2):
                self.mode_switch = self.stabilize

            # Axis controls
            self.roll_pwm = self._scale_joystick(joystick.get_axis(3))
            self.pitch_pwm = self._scale_joystick(joystick.get_axis(4))
            self.throttle_pwm = self._scale_joystick(-joystick.get_axis(1))
            self.yaw_pwm = self._scale_joystick(joystick.get_axis(0))

            Logger.debug(f"PWM inputs :- Roll: {self.roll_pwm}, Pitch: {self.pitch_pwm}, Throttle: {self.throttle_pwm}, Yaw: {self.yaw_pwm}")
            time.sleep(0.1)

    def stop(self):
        self.running = False
        self.thread.join()

