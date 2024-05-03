import numpy as np

class KalmanFilter:
    def __init__(self, initial_state):
        m = 0.03  # mass of the drone in kg
        self.g = 9.81  # gravity
        kvx = 0.03
        kvy = 0.04
        kvz = 0

        self.ang_max = 15
        self.ang_max_rad = np.deg2rad(self.ang_max)

        # Matrices A, C
        A = np.array([[0, 0, 0, 1, 0, 0],
                    [0, 0, 0, 0, 1, 0],
                    [0, 0, 0, 0, 0, 1],
                    [0, 0, 0, -kvx/m, 0, 0],
                    [0, 0, 0, 0, -kvy/m, 0],
                    [0, 0, 0, 0, 0, -kvz/m]])
        
        C = np.array([[1, 0, 0, 0, 0, 0],
                      [0, 1, 0, 0, 0, 0],
                      [0, 0, 1, 0, 0, 0],
                      [0, 0, 0, 1, 0, 0],
                      [0, 0, 0, 0, 1, 0],
                      [0, 0, 0, 0, 0, 1]])
        
        # Parameters
        r_xyz = 0.1
        r_xyz_dot = 0.05
        r_optitrack = 0.01
        r_kalman_drone = 0.1

        # Etate matrix
        Rw = np.block([
            [r_xyz * np.eye(3), np.zeros((3, 3))],
            [np.zeros((3, 3)), r_xyz_dot * np.eye(3)]
        ])

        # Measurement matrix
        Rv = np.block([
            [r_optitrack * np.eye(3), np.zeros((3, 3))],
            [np.zeros((3, 3)), r_kalman_drone * np.eye(3)]
        ])

        self.A = A
        self.C = C
        self.Rw = Rw
        self.Rv = Rv
        self.state_estimate = initial_state
        self.error_covariance = np.array([[1, 0, 0, 0, 0, 0],
                                          [0, 1, 0, 0, 0, 0],
                                          [0, 0, 1, 0, 0, 0],
                                          [0, 0, 0, 0.1, 0, 0],
                                          [0, 0, 0, 0, 0.1, 0],
                                          [0, 0, 0, 0, 0, 0.1]])
        

    def update(self, measurement):

        # Innovation
        innovation = measurement - np.dot(self.C, self.state_estimate)
        innovation_covariance = self.C @ self.error_covariance @ self.C.T + self.Rv

        # Correction
        kalman_gain = self.error_covariance @ self.C.T @ np.linalg.inv(innovation_covariance)
        self.state_estimate = self.state_estimate + kalman_gain @ innovation
        self.error_covariance = (np.eye(self.A.shape[0]) - kalman_gain @ self.C) @ self.error_covariance

        # Prediction
        self.state_estimate = self.A @ self.state_estimate
        self.error_covariance = self.A @ self.error_covariance @ self.A.T + self.Rw

        return self.state_estimate

# Example usage:
# A = ... # Define A
# C = ... # Define C
# Rw = ... # Define Rw
# Rv = ... # Define Rv
# initial_state = ... # Define initial state
# initial_covariance = ... # Define initial covariance
# kf = KalmanFilter(A, C, Rw, Rv, initial_state, initial_covariance)
# for measurement in measurements:  # Assuming measurements is an iterable of numpy arrays
#     updated_state = kf.update(measurement)
