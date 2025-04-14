import threading
import time
from pymavlink import mavutil

class MavlinkSensor:
    def __init__(self, connection_str='tcp:127.0.0.1:5760'):
        self.connection = mavutil.mavlink_connection(connection_str)
        self.roll_angle = 0.0
        self.roll_rate = 0.0
        self.pitch_angle = 0.0
        self.pitch_rate = 0.0
        self.yaw_angle = 0.0
        self.yaw_rate = 0.0
        self.running = True
        self.listener_thread = threading.Thread(target=self._listen_loop, daemon=True)
        self.listener_thread.start()

    def _listen_loop(self):
        while self.running:
            msg = self.connection.recv_match(type='ATTITUDE', blocking=True, timeout=1.0)
            if msg:
                self.roll_angle = msg.roll * 57.3  # radians to degrees
                self.roll_rate = msg.rollspeed * 57.3  # rad/s to deg/s
                
                self.pitch_angle = msg.pitch * 57.3  # radians to degrees
                self.pitch_rate = msg.pitchspeed * 57.3  # rad/s to deg/s
                
                self.yaw_angle = msg.yaw * 57.3  # radians to degrees
                self.yaw_rate = msg.yawspeed * 57.3  # rad/s to deg/s

    def read_roll(self):
        return self.roll_angle

    def read_roll_rate(self):
        return self.roll_rate
    
    def read_pitch(self):
        return self.pitch_angle

    def read_pitch_rate(self):
        return self.pitch_rate

    def read_yaw(self):
        return self.yaw_angle

    def read_yaw_rate(self):
        return self.yaw_rate

    def stop(self):
        self.running = False
        self.listener_thread.join()

