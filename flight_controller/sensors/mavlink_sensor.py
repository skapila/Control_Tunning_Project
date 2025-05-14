import threading
import time
from pymavlink import mavutil
from utils.logger import Logger

class MavlinkSensor:
    def __init__(self, connection_str='tcp:127.0.0.1:5760'):
        self.connection = mavutil.mavlink_connection(connection_str)
        self.roll_angle = 0.0
        self.roll_rate = 0.0
        self.pitch_angle = 0.0
        self.pitch_rate = 0.0
        self.yaw_angle = 0.0
        self.yaw_rate = 0.0
        self.yaw_radian =0.0
        self.attitude_msg = {}
        self.altitude = 0.0
        self.altitude_msg = {}
        self.lat = 0.0
        self.lon = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0

        self.running = True
        self.listener_thread = threading.Thread(target=self._listen_loop, daemon=True)
        self.listener_thread.start()

    def send_mavlink_streaming_signal(self):
        Logger.info(f"Requesting data stream...")
        self.connection.mav.request_data_stream_send(
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL, 
            10, 1)

    def send_mavlink_pwm(self, motor1, motor2, motor3, motor4):
        Logger.debug(f"PWM output: {motor1}, {motor2}, {motor3}, {motor4}")
        self.connection.mav.rc_channels_override_send(
            self.connection.target_system,
            self.connection.target_component,
            motor2, motor4, motor1, motor3,
            0, 0, 0, 0)

    def _listen_loop(self):
        self.send_mavlink_streaming_signal()
        Logger.info(f"Listening to MAVLink messages...")
        while self.running:
            msg = self.connection.recv_match(blocking=True)
            if msg is None:
                continue
            try:
                msg = msg.to_dict()
                if msg['mavpackettype'] == "ATTITUDE":
                    self.roll_angle = msg['roll'] * 57.3
                    self.roll_rate = msg['rollspeed'] * 57.3
                    self.pitch_angle = msg['pitch'] * 57.3
                    self.pitch_rate = msg['pitchspeed'] * 57.3
                    self.yaw_radian= msg['yaw']
                    self.yaw_angle = msg['yaw'] * 57.3
                    self.yaw_rate = msg['yawspeed'] * 57.3
                    self.attitude_msg = {
                        'roll_angle': self.roll_angle,
                        'roll_rate': self.roll_rate,
                        'pitch_angle': self.pitch_angle,
                        'pitch_rate': self.pitch_rate,
                        'yaw_angle': self.yaw_angle,
                        'yaw_rate': self.yaw_rate
                    }

                elif msg['mavpackettype'] == "VFR_HUD":
                    self.altitude = msg['alt']
                    self.altitude_msg = {'alt': self.altitude}

                elif msg['mavpackettype'] == "GLOBAL_POSITION_INT":
                    self.lat = msg['lat'] / 1e7
                    self.lon = msg['lon'] / 1e7

                elif msg['mavpackettype'] == "LOCAL_POSITION_NED":
                    self.vx = msg['vx']
                    self.vy = msg['vy']
                    self.vz = msg['vz']
                    print("====="+str(self.vx)+"========")                

            except Exception as e:
                Logger.error(f"Exception caught: {e}")
               

    def read_roll(self): return self.roll_angle
    def read_roll_rate(self): return self.roll_rate
    def read_pitch(self): return self.pitch_angle
    def read_pitch_rate(self): return self.pitch_rate
    def read_yaw(self): return self.yaw_radian
    def read_yaw_rate(self): return self.yaw_rate
    def read_alt(self): return self.altitude

    def read_latlon(self): return (self.lat, self.lon)
    def read_velocity_ned(self): return (self.vx, self.vy, self.vz)

    def stop(self):
        self.running = False
        self.listener_thread.join()
