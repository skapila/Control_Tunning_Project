import numpy as np

class AttitudeEKF:
    def __init__(self, gyro_noise_std=0.01, initial_cov=0.01 * np.eye(4)):
        """
        Initialize EKF for orientation estimation (prediction only).
        
        Parameters:
        - gyro_noise_std: float or (3,) array, standard deviation of gyro noise (rad/s)
        - initial_cov: (4,4) array, initial state covariance matrix
        """
        self.q = np.array([1.0, 0.0, 0.0, 0.0])  # initial quaternion
        self.P = initial_cov.copy()
        self.gyro_noise_std = (
            np.array(gyro_noise_std) if np.ndim(gyro_noise_std) > 0 else np.full(3, gyro_noise_std)
        )

    def quaternion_to_euler(self, q=None, degrees=False):
        """
        Convert quaternion [w, x, y, z] to Euler angles (roll, pitch, yaw).
        Returns (roll, pitch, yaw) in radians or degrees.
        If q is None, uses the current state quaternion.
        """
        if q is None:
            q = self.q
        q0, q1, q2, q3 = q

        # Roll (x-axis rotation) — mapped to [0, 2π]
        sinr_cosp = 2.0 * (q0*q1 + q2*q3)
        cosr_cosp = 1.0 - 2.0 * (q1*q1 + q2*q2)
        roll = np.arctan2(sinr_cosp, cosr_cosp) % (2 * np.pi)

        # Pitch (y-axis rotation)
        sinp = 2.0 * (q0*q2 - q3*q1)
        if abs(sinp) >= 1:
            pitch = np.pi/2 * np.sign(sinp)
        else:
            pitch = np.arcsin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2.0 * (q0*q3 + q1*q2)
        cosy_cosp = 1.0 - 2.0 * (q2*q2 + q3*q3)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        angles = np.array([roll, pitch, yaw])
        return np.degrees(angles) if degrees else angles

    def omega_matrix(self, omega):
        wx, wy, wz = omega
        return np.array([
            [ 0.0, -wx, -wy, -wz],
            [ wx,  0.0,  wz, -wy],
            [ wy, -wz,  0.0,  wx],
            [ wz,  wy, -wx,  0.0]
        ])

    def normalize_quaternion(self, q):
        return q / np.linalg.norm(q)

    def compute_W(self, q):
        q0, q1, q2, q3 = q
        return 0.5 * np.array([
            [-q1, -q2, -q3],
            [ q0, -q3,  q2],
            [ q3,  q0, -q1],
            [-q2,  q1,  q0]
        ])

    def compute_process_noise_Q(self, q):
        gyro_cov = np.diag(np.square(self.gyro_noise_std))
        W = self.compute_W(q)
        return W @ gyro_cov @ W.T

    def predict(self, omega, dt):
        """
        Perform EKF prediction step with current state.
        
        Parameters:
        - omega: (3,) array, gyroscope data in rad/s
        - dt: float, timestep in seconds

        Returns:
        - q: predicted quaternion
        - P: predicted covariance matrix
        """
        omega = np.asarray(omega)
        Omega = self.omega_matrix(omega)

        # Quaternion derivative
        q_dot = 0.5 * Omega @ self.q
        self.q = self.normalize_quaternion(self.q + q_dot * dt)

        F = np.eye(4) + 0.5 * Omega * dt
        Q = self.compute_process_noise_Q(self.q)
        self.P = F @ self.P @ F.T + Q

        return self.q.copy(), self.P.copy()

    def update_accel_only(self, acc, R_acc=None):
        """
        EKF update step using only accelerometer data (no magnetometer).
        Parameters:
        - acc: (3,) Accelerometer measurement in m/s²
        - R_acc: (3x3) Measurement noise covariance for accel (default: 0.1 * I)
        """
        acc = np.asarray(acc)
        acc = acc / np.linalg.norm(acc)  # Normalize

        if R_acc is None:
            R_acc = 0.1 * np.eye(3)

        # Snapshot quaternion before update
        q = self.q.copy()
        q0, q1, q2, q3 = q

        # Expected accelerometer reading = -R^T @ [0,0,1] (specific force, NED, Z-down)
        # At identity quaternion this gives [0, 0, -1] as expected for a static FRD drone
        g_hat = np.array([
            -2*(q1*q3 - q0*q2),
            -2*(q0*q1 + q2*q3),
            -(q0**2 - q1**2 - q2**2 + q3**2)
        ])

        # Residual (3,)
        y = acc - g_hat

        # Jacobian H: (3x4) partial of g_hat w.r.t. [q0,q1,q2,q3]
        H = np.array([
            [ 2*q2, -2*q3,  2*q0, -2*q1],
            [-2*q1, -2*q0, -2*q3, -2*q2],
            [-2*q0,  2*q1,  2*q2, -2*q3]
        ])

        # Kalman gain
        S = H @ self.P @ H.T + R_acc     # (3x3)
        K = self.P @ H.T @ np.linalg.inv(S)  # (4x3)

        # State correction
        self.q += K @ y
        self.q = self.normalize_quaternion(self.q)

        # Covariance update
        self.P = (np.eye(4) - K @ H) @ self.P
        return self.q.copy(), self.P.copy(), S.copy()

    def update(self, acc, mag, R_acc=None, R_mag=None):
        """
        EKF correction step using accelerometer and magnetometer in NED frame.

        Parameters:
        - acc: (3,) Accelerometer in m/s² (NED, Z+ down)
        - mag: (3,) Magnetometer in any consistent unit (Gauss, µT, etc.)
        - R_acc: (3x3) Accelerometer noise covariance
        - R_mag: (3x3) Magnetometer noise covariance
        """
        acc = np.asarray(acc)
        mag = np.asarray(mag)

        if R_acc is None:
            R_acc = 0.1 * np.eye(3)
        if R_mag is None:
            R_mag = 0.1 * np.eye(3)

        q = 1 * self.q
        q0, q1, q2, q3 = q

        # Normalize measurements
        acc = acc / np.linalg.norm(acc)
        mag = mag / np.linalg.norm(mag)

        # Expected accelerometer reading = -R^T @ [0,0,1] (specific force, NED, Z-down)
        g_hat = np.array([
            -2*(q1*q3 - q0*q2),
            -2*(q0*q1 + q2*q3),
            -(q0**2 - q1**2 - q2**2 + q3**2)
        ])

        # Expected magnetic field vector in NED (approximate: reference = [0,1,0])
        m_hat = np.array([
            2*(q1*q2 + q0*q3),
            q0**2 - q1**2 + q2**2 - q3**2,
            2*(q2*q3 - q0*q1)
        ])

        y = np.concatenate((acc - g_hat, mag - m_hat))  # shape (6,)

        # Jacobian H: partial of [g_hat; m_hat] w.r.t. [q0,q1,q2,q3]
        H = np.array([
            [ 2*q2, -2*q3,  2*q0, -2*q1],
            [-2*q1, -2*q0, -2*q3, -2*q2],
            [-2*q0,  2*q1,  2*q2, -2*q3],

            [ 2*q3,   2*q2,  2*q1,  2*q0],
            [ 2*q0,  -2*q1,  2*q2, -2*q3],
            [-2*q1,  -2*q0,  2*q3,  2*q2]
        ])

        # Measurement noise matrix
        R = np.block([
            [R_acc, np.zeros((3, 3))],
            [np.zeros((3, 3)), R_mag]
        ])  # shape (6, 6)

        # Kalman gain
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)

        # Update quaternion and covariance
        self.q += K @ y
        self.q = self.normalize_quaternion(self.q)
        self.P = (np.eye(4) - K @ H) @ self.P    
        return self.q.copy(), self.P.copy(), S.copy()