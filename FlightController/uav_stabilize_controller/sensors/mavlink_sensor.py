import threading
import time
from pymavlink import mavutil
from utils.logger import Logger


# This class is responsible for reading and sending sensor data from a MAVLink connection.
class MavlinkSensor:
    def __init__(self, connection_str='tcp:127.0.0.1:5760'):
        self.connection = mavutil.mavlink_connection(connection_str)
        self.roll_angle = 0.0
        self.roll_rate = 0.0
        self.pitch_angle = 0.0
        self.pitch_rate = 0.0
        self.yaw_angle = 0.0
        self.yaw_rate = 0.0
        self.attitude_msg = {}
        
        self.altitude =  0.0
        self.altitude_msg = {}
        
        self.running = True
        self.listener_thread = threading.Thread(target=self._listen_loop, daemon=True)
        self.listener_thread.start()


    def send_mavlink_streaming_signal(self):
        
        # Request data stream
        Logger.info(f"Requesting data stream...")
        self.connection.mav.request_data_stream_send(
        self.connection.target_system,
        self.connection.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL, 
        10, 1)

    def send_mavlink_pwm(self, motor1, motor2, motor3, motor4):

        # Send RC command via MAVLink
        
        Logger.debug(f"PWM output: {motor1}, {motor2}, {motor3}, {motor4}")
        self.connection.mav.rc_channels_override_send(
        self.connection.target_system,  # Target system ID
        self.connection.target_component,  # Target component ID
        motor2,motor4,motor1,motor3,  # motor channels 1-4[2,4,1,3]
        0, 0, 0, 0  # Unused channels
        )
   
    def _listen_loop(self):
       
        # Sending mavlink steaming signal
        self.send_mavlink_streaming_signal()
       
        # Start listening to MAVLink messages

        Logger.info(f"Listening to MAVLink messages...")
        while self.running:
            msg = self.connection.recv_match(blocking=True).to_dict()
            try:    
             if(msg['mavpackettype']=="ATTITUDE"):

                self.roll_angle = msg['roll'] * 57.3  # radians to degrees
                self.roll_rate = msg['rollspeed'] * 57.3  # rad/s to deg/s
            
                self.pitch_angle = msg['pitch'] * 57.3  # radians to degrees
                self.pitch_rate = msg['pitchspeed'] * 57.3  # rad/s to deg/s
                
                self.yaw_angle = msg['yaw'] * 57.3  # radians to degrees
                self.yaw_rate = msg['yawspeed'] * 57.3  # rad/s to deg/s
                 
                self.attitude_msg= {'roll_angle':self.roll_angle, 'roll_rate':self.roll_rate, 'pitch_angle':self.pitch_angle, 'pitch_rate':self.pitch_rate, 'yaw_angle': self.yaw_angle, 'yaw_rate':self.yaw_rate}
                #Logger.debug(f"Attitude Message : {self.attitude_msg}")
             
             elif(msg['mavpackettype']=="VFR_HUD"):
             
                self.altitude = msg['alt']
                self.altitude_msg = {'alt': self.altitude}
                #Logger.debug(f"Attitude Message : {self.altitude_msg}")
             
            except: 
              pass
    

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

    def read_alt(self):
        return self.altitude