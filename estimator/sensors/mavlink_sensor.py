import threading
import time
from pymavlink import mavutil
from utils.logger import Logger

class MavlinkSensor:
    def __init__(self, connection_str='tcp:127.0.0.1:5762'):
        self.connection = mavutil.mavlink_connection(connection_str)
        self.roll_angle = 0.0
        self.roll_rate = 0.0
        self.pitch_angle = 0.0
        self.pitch_rate = 0.0
        self.yaw_angle = 0.0
        self.yaw_rate = 0.0
        self.yaw_radian =0.0
        self.altitude = 0.0
        self.lat = 0.0
        self.lon = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.x=0.0
        self.y=0.0
        self.z=0.0
        self.ax=0.0
        self.ay=0.0
        self.az=0.0
        self.gx=0.0
        self.gy=0.0
        self.gz=0.0
        self.mx=0.0
        self.my=0.0
        self.mz=0.0
        self.gps_vel_horiz = 0.0  # Horizontal GPS velocity in m/s
        self.accel_scale = 9.81/1000 # m/s²
        self.gyro_scale = 1 / 1000  # RAW_IMU gyro: millirad/s → rad/s
        self.incoming_imu_flag=0
        self.incoming_mag_flag=0
        self.incoming_gps_flag=0
        self.running = True
        self.listener_thread = threading.Thread(target=self._listen_loop, daemon=True)
        self.listener_thread.start()

    def send_mavlink_streaming_signal(self):
        Logger.info(f"Requesting RAW_IMU stream at 25 Hz...")
        self.connection.mav.request_data_stream_send(
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL, 25, 1)  # 25 Hz, start streaming
          
           
    def request_raw_imu_25hz(self, interval_us = 40000):
        """MAVLink v2: ask the vehicle to send only RAW_IMU at exactly 25 Hz."""
        Logger.info("Requesting RAW_IMU only at 25 Hz via SET_MESSAGE_INTERVAL...")
        self.connection.mav.command_long_send(
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,                                              # confirmation
            mavutil.mavlink.MAVLINK_MSG_ID_RAW_IMU,         # message ID 27
            interval_us,                                    # interval µs → 25 Hz
            0, 0, 0, 0, 0)

    def send_mavlink_pwm(self, motor1, motor2, motor3, motor4):
        Logger.debug(f"PWM output: {motor1}, {motor2}, {motor3}, {motor4}")
        self.connection.mav.rc_channels_override_send(
            self.connection.target_system,
            self.connection.target_component,
            motor2, motor4, motor1, motor3,
            0, 0, 0, 0)
    

    def _listen_loop(self):
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
                   
                elif msg['mavpackettype'] == "VFR_HUD":
                    self.altitude = msg['alt']
                   
                elif msg['mavpackettype'] == "GLOBAL_POSITION_INT":
                    self.lat = msg['lat'] / 1e7
                    self.lon = msg['lon'] / 1e7

                elif msg['mavpackettype'] == "LOCAL_POSITION_NED":
                    self.vx = msg['vx']
                    self.vy = msg['vy']
                    self.vz = msg['vz']
                    self.x =  msg['x']
                    self.y =  msg['y']
                    self.z =  msg['z']
            
                elif msg['mavpackettype'] == "GPS_RAW_INT":
                   self.lat = msg['lat'] / 1e7              # degrees
                   self.lon = msg['lon'] / 1e7              # degrees
                   self.alt = msg['alt'] / 1000.0           # meters
                   self.gps_vel_horiz = msg['vel'] / 100.0  # m/s (2D ground speed)
                   
                elif msg['mavpackettype'] == "RAW_IMU": 
                   # Convert accelerometer
                   self.ax= msg['xacc'] * self.accel_scale
                   self.ay = msg['yacc'] * self.accel_scale
                   self.az = msg['zacc'] * self.accel_scale

                   # Convert gyroscope
                   self.gx = msg['xgyro'] * self.gyro_scale
                   self.gy = msg['ygyro'] * self.gyro_scale
                   self.gz = msg['zgyro'] * self.gyro_scale
                   self.incoming_imu_flag=1

                   # calculate magnetometer values
                   self.mx = msg['xmag'] * 1e-3  # milliGauss → Gauss
                   self.my = msg['ymag'] * 1e-3
                   self.mz = msg['zmag'] * 1e-3  
                   self.incoming_mag_flag=1
                  
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
    def read_gyro_raw(self): return (self.gx,self.gy,self.gz)
    def read_accel_raw(self): return (self.ax,self.ay,self.az)
    def read_magnetometer_raw(self): return (self.mx,self.my,self.mz)
    def read_gps_raw(self): return(self.lat,self.lon,self.alt)  
    def read_pos_ardupilot_ekf(self):return(self.x,self.y,self.z)
    def read_euler_ardupilot_ekf(self): return(self.roll_angle,self.pitch_angle,self.yaw_angle)
    def read_velocity_ardupilot_ekf(self): return(self.vx,self.vy,self.vz)
    def is_incoming_gps_message(self):  return(self.incoming_gps_flag)
    def is_incoming_mag_message(self):  return(self.incoming_mag_flag)
    def is_incoming_imu_message(self):  return(self.incoming_imu_flag)         
    def reset_incoming_gps_flag(self): self.incoming_gps_flag=0
    def reset_incoming_mag_flag(self): self.incoming_mag_flag=0
    def reset_incoming_imu_flag(self): self.incoming_imu_flag=0           
              
    def stop(self):
        self.running = False
        self.listener_thread.join()
