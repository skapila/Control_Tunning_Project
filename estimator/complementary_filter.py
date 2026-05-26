import numpy as np


class ComplementaryFilter:
    """
    Complementary filter for roll and pitch estimation.

    Blends gyroscope integration (fast, drifts over time) with
    accelerometer tilt estimate (slow, noisy) using a fixed alpha weight.

    alpha close to 1.0 → trust gyro more (less accel correction)
    alpha close to 0.0 → trust accel more (more correction, more noise)
    """

    def __init__(self, alpha: float = 0.98):
        if not (0.0 < alpha < 1.0):
            raise ValueError("alpha must be in (0, 1)")
        self.alpha = alpha
        self.roll  = 0.0   # radians
        self.pitch = 0.0   # radians

    def update(self, accel: np.ndarray, gyro: np.ndarray, dt: float) -> tuple[float, float]:
        """
        Update filter with one IMU sample.

        Parameters
        ----------
        accel : (3,) array  [ax, ay, az] in m/s²
        gyro  : (3,) array  [gx, gy, gz] in rad/s
        dt    : float       time step in seconds

        Returns
        -------
        (roll, pitch) in radians
        """
        ax, ay, az = accel
        gx, gy, _  = gyro

        # Accel-based tilt (valid only when |a| ≈ g, i.e. no linear accel)
        accel_roll  = np.arctan2(ay, az)
        accel_pitch = np.arctan2(-ax, np.sqrt(ay**2 + az**2))

        # Gyro integration
        gyro_roll  = self.roll  + gx * dt
        gyro_pitch = self.pitch + gy * dt

        # Complementary blend
        self.roll  = self.alpha * gyro_roll  + (1.0 - self.alpha) * accel_roll
        self.pitch = self.alpha * gyro_pitch + (1.0 - self.alpha) * accel_pitch

        return self.roll, self.pitch

    def get_euler_degrees(self) -> tuple[float, float]:
        """Return current (roll, pitch) in degrees."""
        return np.degrees(self.roll), np.degrees(self.pitch)

    def reset(self, roll: float = 0.0, pitch: float = 0.0) -> None:
        """Reset state, optionally to a known initial angle (radians)."""
        self.roll  = roll
        self.pitch = pitch
