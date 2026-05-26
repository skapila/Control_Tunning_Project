from attitude_ekf import AttitudeEKF
from complementary_filter import ComplementaryFilter
from sensors.mavlink_sensor import MavlinkSensor
from utils.logger import Logger
from collections import deque
import numpy as np
import time

TARGET_HZ  = 25
DT_TARGET  = 1.0 / TARGET_HZ   # 0.04 s
AVG_WINDOW = 25                 # samples for rolling average frequency

# --- Connection ---
sensors = MavlinkSensor('/dev/ttyACM0')

# Request RAW_IMU only at 25 Hz via SET_MESSAGE_INTERVAL (MAVLink v2)
time.sleep(1.0)   # give heartbeat time to arrive before sending command
sensors.request_raw_imu_25hz(DT_TARGET*1000*1000)  # convert to ms

# --- Wait for first valid IMU sample ---
Logger.info("Waiting for IMU data...")
while not sensors.is_incoming_imu_message():
    pass
accel_init = sensors.read_accel_raw()
Logger.info(f"Initial accel: {accel_init}")

# --- Accel mounting correction (sensor rotated 45° clockwise on board) ---
# Apply inverse rotation (+45° CCW around Z) to map sensor frame → board frame
_a = np.radians(45.0)
R_MOUNT = np.array([
    [ np.cos(_a), -np.sin(_a), 0.0],
    [ np.sin(_a),  np.cos(_a), 0.0],
    [        0.0,         0.0, 1.0],
])

# --- EKF + Complementary filter setup ---
ekf = AttitudeEKF(gyro_noise_std=[0.01, 0.01, 0.01])
cf  = ComplementaryFilter(alpha=0.98)
sensors.reset_incoming_imu_flag()

Logger.info(f"Running EKF at {TARGET_HZ} Hz (fixed consumer loop)...")

last_imu_time  = time.monotonic()
interval_buf   = deque(maxlen=AVG_WINDOW)   # rolling window of intervals (s)

# --- Main 25 Hz loop ---
while True:
    t0 = time.monotonic()

    if sensors.is_incoming_imu_message():
        accel = R_MOUNT @ np.array(sensors.read_accel_raw())   # m/s², board frame
        gyro  = R_MOUNT @ np.array(sensors.read_gyro_raw())  # rad/s, board frame
        sensors.reset_incoming_imu_flag()

        # --- Frequency tracking ---
        now          = time.monotonic()
        interval_s   = now - last_imu_time
        last_imu_time = now

        interval_buf.append(interval_s)
        instant_hz = 1.0 / interval_s if interval_s > 0 else 0.0
        avg_hz     = 1.0 / np.mean(interval_buf) if interval_buf else 0.0

        # --- EKF predict + update ---
        q, P            = ekf.predict(gyro, DT_TARGET)
        q_est, P_est, S = ekf.update_accel_only(accel)

        roll, pitch, yaw = ekf.quaternion_to_euler(degrees=True)
        roll  = roll - 180.0
        pitch = -pitch

        # --- Complementary filter ---
        cf.update(accel, gyro, DT_TARGET)
        cf_roll, cf_pitch = cf.get_euler_degrees()

        Logger.info(
            f"EKF  Roll={roll:7.2f}°  Pitch={pitch:7.2f}°  Yaw={yaw:7.2f}°  "
            f"| CF Roll={cf_roll:7.2f}°  Pitch={cf_pitch:7.2f}°  "
            f"| instant={instant_hz:5.1f} Hz  avg={avg_hz:5.1f} Hz  interval={interval_s*1000:.1f} ms"
        )
        Logger.debug(
            f"accel={np.round(accel, 3)}  gyro={np.round(gyro, 4)}"
        )

    # Pace to TARGET_HZ
    elapsed    = time.monotonic() - t0
    sleep_time = max(0.0, DT_TARGET - elapsed)
    Logger.debug(f"Loop sleep: {sleep_time*1000:.2f} ms")
    time.sleep(sleep_time)
