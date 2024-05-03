import numpy as np
import control

def normalize_angle(angle):
    while angle > np.pi:
        angle -= 2 * np.pi
    while angle <= -np.pi:
        angle += 2 * np.pi
    return angle

class Bebop:
    def __init__(self, pose=None, desired=None, gains=None, max_variables=None):
        # pose = [    x  ,    y  ,    z  ,    theta  ,   phi   ,    psi  , 
        #          x_dot ,  y_dot,  z_dot,  theta_dot,  phi_dot,  psi_dot]
        pose = [0]*12 if pose is None else pose

        # desired = [    x  ,    y  ,    z  ,    theta  ,   phi   ,    psi  , 
        #             x_dot ,  y_dot,  z_dot,  theta_dot,  phi_dot,  psi_dot, 
        #             x_ddot, y_ddot, z_ddot, theta_ddot, phi_ddot, psi_ddot]
        desired = [0]*18 if desired is None else desired

        # gains = [Kp_Theta, Kp_Phi, kd_Theta, Kd_Phi, Kp_z_dot, Kd_Ksi_dot]
        gains =[4.0, 4.0, 4.0, 4.0, 1.35, 1.33] if gains is None else gains

        # max_variables = [Theta_max(rad), Phi_max(rad), z_dot_max(m/s), ksi_dot_max(rad/s)]
        max_variables=[(5*np.pi)/180, (5*np.pi)/180, 1, 1.75] if max_variables is None else max_variables

        self.set_pose(pose)
        self.set_desired(desired)
        self.set_gains(gains)
        self.set_max_variables(max_variables)

    def set_pose(self, pose):
        self.pose = self._unpack_array(pose, 12, "Pose")

    def set_desired(self, desired):
        self.desired = self._unpack_array(desired, 18, "Desired")

    def set_gains(self, gains):
        self.gains = self._unpack_array(gains, 6, "Gains")

    def set_max_variables(self, max_variables):
        self.max_variables = self._unpack_array(max_variables, 4, "Max Variables")
    
    def _unpack_array(self, arr, expected_length, name):
        # Check if array has expected length
        if len(arr) != expected_length:
            raise ValueError(f"{name} size is incorrect. Expected size: {expected_length}. Got: {len(arr)}")
        return np.array(arr)

    def compute_controller(self):
        # Prepare parameters
        kp = np.array([[self.gains[0],     0     ],
                       [   0    ,   self.gains[1]]])

        kd = np.array([[self.gains[2],     0     ],
                       [   0    ,   self.gains[3]]])
        
        Kp_z_psi = np.array([[self.gains[4],     0     ],
                            [   0    ,   self.gains[5]]])

        theta_max, phi_max, z_dot_max, psi_dot_max = self.max_variables

        # Compute theta and phi controller commands
        u_theta, u_phi = self._compute_u_theta_phi(kp, kd, theta_max, phi_max)

        # Compute z_dot and psi_dot controller commands
        u_z_dot, u_psi_dot= self._compute_u_z_dot_psi_dot(Kp_z_psi, z_dot_max, psi_dot_max)
        
        # Normalizing inputs to a maximum value of 1.0
        u_theta   = np.sign(u_theta)   if np.abs(u_theta)  >1.0 else u_theta
        u_phi     = np.sign(u_phi)     if np.abs(u_phi)    >1.0 else u_phi
        u_z_dot   = np.sign(u_z_dot)   if np.abs(u_z_dot)  >1.0 else u_z_dot
        u_psi_dot = np.sign(u_psi_dot) if np.abs(u_psi_dot)>1.0 else u_psi_dot

        return u_theta, u_phi, u_z_dot, u_psi_dot

    def _compute_u_theta_phi(self, kp, kd, theta_max, phi_max):
        g = 9.8 # gravity acceleration constant

        # Compute tilde values
        X_til     = self.desired[:2] - self.pose[:2]
        X_dot_til = self.desired[6:8] - self.pose[6:8]

        # Compute reference values
        X_ddot_ref = self.desired[12:14] + np.dot(kd, X_dot_til) + np.dot(kp, X_til)

        # Compute normalization and rotation matrices
        normalization_theta_phi = np.array([[1/(g*theta_max), 0], 
                                            [0, 1/(g*phi_max)]])

        rotation_matrix_theta_phi = np.array([[np.cos(self.pose[5]) , np.sin(self.pose[5])], 
                                              [-np.sin(self.pose[5]), np.cos(self.pose[5])]])

        # Compute theta and phi controller commands
        u_theta, u_phi = np.dot(normalization_theta_phi, np.dot(rotation_matrix_theta_phi, X_ddot_ref))

        return u_theta, u_phi

    def _compute_u_z_dot_psi_dot(self, Kp_z_psi, z_dot_max, psi_dot_max):
        # Normalize the desired psi and the current psi to the range [-pi, pi].
        desired_psi = normalize_angle(self.desired[5])
        current_psi = normalize_angle(self.pose[5])

        # Compute the shortest angular distance.
        delta_psi = normalize_angle(desired_psi - current_psi)

        # Compute tilde values, using the shortest angular distance.
        Z_PSI_til = np.array([self.desired[2] - self.pose[2], delta_psi])

        # Compute reference values
        z_dot_ref, psi_dot_ref = np.array([self.desired[8], self.desired[11]]) + np.dot(Kp_z_psi, Z_PSI_til)

        # Compute z_dot and psi_dot controller commands
        u_z_dot = z_dot_ref / z_dot_max
        u_psi_dot = psi_dot_ref / psi_dot_max

        return u_z_dot, u_psi_dot

    

class Tello:
    def __init__(self, pose=None, desired=None, gains=None, max_variables=None, linear_drag=None):
        # pose = [    x  ,    y  ,    z  ,    theta  ,   phi   ,    psi  , 
        #          x_dot ,  y_dot,  z_dot,  theta_dot,  phi_dot,  psi_dot]
        pose = [0]*12 if pose is None else pose

        # desired = [    x  ,    y  ,    z  ,    theta  ,   phi   ,    psi  , 
        #             x_dot ,  y_dot,  z_dot,  theta_dot,  phi_dot,  psi_dot, 
        #             x_ddot, y_ddot, z_ddot, theta_ddot, phi_ddot, psi_ddot]
        desired = [0]*18 if desired is None else desired

        # gains = [Kp_Theta, Kp_Phi, kd_Theta, Kd_Phi, Kp_z_dot, Kd_Ksi_dot]
        gains =[2.84, 2.84, 2.84, 2.84, 1.72, 1.5] if gains is None else gains

        # max_variables = [Theta_max(rad), Phi_max(rad), z_dot_max(m/s), ksi_dot_max(rad/s)]
        max_variables=[(5*np.pi)/180, (5*np.pi)/180, 1, 1.75] if max_variables is None else max_variables

        linear_drag = [0.7765, 0.7282] if linear_drag is None else linear_drag

        self.set_pose(pose)
        self.set_desired(desired)
        self.set_gains(gains)
        self.set_max_variables(max_variables)
        self.set_linear_drad(linear_drag)

    def set_pose(self, pose):
        self.pose = self._unpack_array(pose, 12, "Pose")

    def set_desired(self, desired):
        self.desired = self._unpack_array(desired, 18, "Desired")

    def set_gains(self, gains):
        self.gains = self._unpack_array(gains, 6, "Gains")

    def set_max_variables(self, max_variables):
        self.max_variables = self._unpack_array(max_variables, 4, "Max Variables")
    
    def set_linear_drad(self, linear_drag):
        self.linear_drag = self._unpack_array(linear_drag, 2, "Max Variables")

    def _unpack_array(self, arr, expected_length, name):
        # Check if array has expected length
        if len(arr) != expected_length:
            raise ValueError(f"{name} size is incorrect. Expected size: {expected_length}. Got: {len(arr)}")
        return np.array(arr)

    def compute_controller(self):
        # Prepare parameters
        kp = np.array([[self.gains[0],     0     ],
                       [   0    ,   self.gains[1]]])

        kd = np.array([[self.gains[2],     0     ],
                       [   0    ,   self.gains[3]]])
        
        Kp_z_psi = np.array([[self.gains[4],     0     ],
                            [   0    ,   self.gains[5]]])

        theta_max, phi_max, z_dot_max, psi_dot_max = self.max_variables

        linear_drag_x, linear_drag_y = self.linear_drag

        # Compute theta and phi controller commands
        u_theta, u_phi = self._compute_u_theta_phi(kp, kd, theta_max, phi_max, linear_drag_x, linear_drag_y)

        # Compute z_dot and psi_dot controller commands
        u_z_dot, u_psi_dot = self._compute_u_z_dot_psi_dot(Kp_z_psi, z_dot_max, psi_dot_max)

        # Normalizing inputs to a maximum value of 1.0
        u_theta   = np.sign(u_theta)   if np.abs(u_theta)  > 1.0 else u_theta
        u_phi     = np.sign(u_phi)     if np.abs(u_phi)    > 1.0 else u_phi
        u_z_dot   = np.sign(u_z_dot)   if np.abs(u_z_dot)  > 1.0 else u_z_dot
        u_psi_dot = np.sign(u_psi_dot) if np.abs(u_psi_dot)  > 1.0 else u_psi_dot

        return u_theta, u_phi, u_z_dot, u_psi_dot

    def _compute_u_theta_phi(self, kp, kd, theta_max, phi_max, linear_drag_x, linear_drag_y):
        g = 9.8 # gravity acceleration constant

        # Compute tilde values
        X_til     = self.desired[:2] - self.pose[:2]
        X_dot_til = self.desired[6:8] - self.pose[6:8]

        # Compute linear drag
        linear_drag_constants = np.array([[linear_drag_x,      0      ],
                                          [     0,       linear_drag_y]])
        
        linear_drag = np.dot(linear_drag_constants, self.pose[6:8])

        # Compute reference values
        X_ddot_ref = 0*self.desired[12:14] + np.dot(kd, X_dot_til) + np.dot(kp, X_til) + 0*linear_drag

        # Compute normalization and rotation matrices
        normalization_theta_phi = np.array([[1/(g*theta_max), 0], 
                                            [0, 1/(g*phi_max)]])

        rotation_matrix_theta_phi = np.array([[np.cos(self.pose[5]) , np.sin(self.pose[5])], 
                                              [-np.sin(self.pose[5]), np.cos(self.pose[5])]])

        # Compute theta and phi controller commands
        u_theta, u_phi = np.dot(normalization_theta_phi, np.dot(rotation_matrix_theta_phi, X_ddot_ref))

        return u_theta, -u_phi

    def _compute_u_z_dot_psi_dot(self, Kp_z_psi, z_dot_max, psi_dot_max):

        desired_psi = normalize_angle(self.desired[5])
        current_psi = normalize_angle(self.pose[5])

        # Compute the shortest angular distance.
        delta_psi = normalize_angle(desired_psi - current_psi)

        # Compute tilde values, using the shortest angular distance.
        Z_PSI_til = np.array([self.desired[2] - self.pose[2], delta_psi])

        # Compute reference values
        z_dot_ref, psi_dot_ref = np.array([self.desired[8], self.desired[11]]) + np.dot(Kp_z_psi, Z_PSI_til)

        # Compute z_dot and psi_dot controller commands
        u_z_dot = z_dot_ref / z_dot_max
        u_psi_dot = psi_dot_ref / psi_dot_max

        return u_z_dot, u_psi_dot
    

class CrazyflieController:
    def __init__(self, pose=None, desired=None, gains=None, max_variables=None, linear_drag=None):
        # pose = [    x  ,    y  ,    z  ,    theta  ,   phi   ,    psi  , 
        #          x_dot ,  y_dot,  z_dot,  theta_dot,  phi_dot,  psi_dot]
        pose = [0]*12 if pose is None else pose

        # desired = [    x  ,    y  ,    z  ,    theta  ,   phi   ,    psi  , 
        #             x_dot ,  y_dot,  z_dot,  theta_dot,  phi_dot,  psi_dot, 
        #             x_ddot, y_ddot, z_ddot, theta_ddot, phi_ddot, psi_ddot]
        desired = [0]*18 if desired is None else desired

        # gains = [Kp_Theta, Kp_Phi, kd_Theta, Kd_Phi, Kp_z_dot, Kp_Ksi_dot, Kd_z_dot]
        gains =[1.2, 1.2, 1, 1, 1.72, 3, 2] if gains is None else gains

        # max_variables = [Theta_max(rad), Phi_max(rad), z_dot_max(m/s), ksi_dot_max(rad/s)]
        max_variables=[(15*np.pi)/180, (15*np.pi)/180, 1, 1.75] if max_variables is None else max_variables

        linear_drag = [0.7765, 0.7282] if linear_drag is None else linear_drag

        self.set_pose(pose)
        self.set_desired(desired)
        self.set_gains(gains)
        self.set_max_variables(max_variables)
        self.set_linear_drad(linear_drag)

    def set_pose(self, pose):
        self.pose = self._unpack_array(pose, 12, "Pose")

    def set_desired(self, desired):
        self.desired = self._unpack_array(desired, 18, "Desired")

    def set_gains(self, gains):
        self.gains = self._unpack_array(gains, 7, "Gains")

    def set_max_variables(self, max_variables):
        self.max_variables = self._unpack_array(max_variables, 4, "Max Variables")
    
    def set_linear_drad(self, linear_drag):
        self.linear_drag = self._unpack_array(linear_drag, 2, "Max Variables")

    def _unpack_array(self, arr, expected_length, name):
        # Check if array has expected length
        if len(arr) != expected_length:
            raise ValueError(f"{name} size is incorrect. Expected size: {expected_length}. Got: {len(arr)}")
        return np.array(arr)

    def compute_controller(self):
        # Prepare parameters
        kp = np.array([[self.gains[0],     0     ],
                       [   0    ,   self.gains[1]]])

        kd = np.array([[self.gains[2],     0     ],
                       [   0    ,   self.gains[3]]])
        
        Kp_z_psi = np.array([[self.gains[4],     0     ],
                            [   0    ,   self.gains[5]]])

        theta_max, phi_max, z_dot_max, psi_dot_max = self.max_variables

        linear_drag_x, linear_drag_y = self.linear_drag

        # Compute theta and phi controller commands
        u_theta, u_phi = self._compute_u_theta_phi(kp, kd, theta_max, phi_max, linear_drag_x, linear_drag_y)

        # Compute z_dot and psi_dot controller commands
        thrust, u_psi_dot = self._compute_u_thrust_psi_dot(Kp_z_psi, z_dot_max, psi_dot_max)

        # Normalizing inputs to a maximum value of 1.0
        u_theta   = np.sign(u_theta)   if np.abs(u_theta)  > 1.0 else u_theta
        u_phi     = np.sign(u_phi)     if np.abs(u_phi)    > 1.0 else u_phi
        thrust   = np.sign(thrust)   if np.abs(thrust)  > 1.0 else thrust
        u_psi_dot = np.sign(u_psi_dot) if np.abs(u_psi_dot)  > 1.0 else u_psi_dot

        return u_theta, u_phi, thrust, u_psi_dot

    def _compute_u_theta_phi(self, kp, kd, theta_max, phi_max, linear_drag_x, linear_drag_y):
        g = 9.8 # gravity acceleration constant

        # Compute tilde values
        X_til     = self.desired[:2] - self.pose[:2]
        X_dot_til = self.desired[6:8] - self.pose[6:8]

        # Compute linear drag
        linear_drag_constants = np.array([[linear_drag_x,      0      ],
                                          [     0,       linear_drag_y]])
        
        linear_drag = np.dot(linear_drag_constants, self.pose[6:8])

        # Compute reference values
        X_ddot_ref = self.desired[12:14] + np.dot(kd, X_dot_til) + np.dot(kp, X_til) + 0*linear_drag

        # Compute normalization and rotation matrices
        normalization_theta_phi = np.array([[1/(g*theta_max), 0], 
                                            [0, 1/(g*phi_max)]])

        rotation_matrix_theta_phi = np.array([[np.cos(self.pose[5]) , np.sin(self.pose[5])], 
                                              [-np.sin(self.pose[5]), np.cos(self.pose[5])]])

        # Compute theta and phi controller commands
        u_theta, u_phi = np.dot(normalization_theta_phi, np.dot(rotation_matrix_theta_phi, X_ddot_ref))

        return u_theta, -u_phi

    def _compute_u_thrust_psi_dot(self, Kp_z_psi, z_dot_max, psi_dot_max):
        a_max = 15
        g = 9.81
        conversao_digital = 1/a_max
        desired_psi = normalize_angle(self.desired[5])
        current_psi = normalize_angle(self.pose[5])

        # Compute the shortest angular distance.
        delta_psi = normalize_angle(desired_psi - current_psi)

        # Compute tilde values, using the shortest angular distance.
        Z_PSI_til = np.array([self.desired[2] - self.pose[2], delta_psi])

        # Compute reference values

        # zref = A.pPos.dXd(9) + 2*(A.pPos.Xd(9) - A.pPos.X(9)) + 4*(A.pPos.Xd(3) - A.pPos.X(3));
        zref = 2*(- vel[2]) + 4*(self.desired_pose[2] - pose[2])
        
        z_dot_ref = self.desired[8] + 2
        z_dot_ref, psi_dot_ref = np.array([self.desired[8], self.desired[11]]) + np.dot(Kp_z_psi, Z_PSI_til)

        thrust = (z_dot_ref + g)/(np.cos(self.pose[3])*np.cos(self.pose[4]))*conversao_digital

        # Compute z_dot and psi_dot controller commands
        u_psi_dot = psi_dot_ref / psi_dot_max

        return thrust, -u_psi_dot
    

class CrazyflieLQR():
    def __init__(self, A=None, B=None, Q=None, R=None) -> None:
        m = 0.03  # mass of the drone in kg
        self.g = 9.81  # gravity
        kvx = 0.03
        kvy = 0.04
        kvz = 0

        self.ang_max = 15
        self.ang_max_rad = np.deg2rad(self.ang_max)

        # Matrices A, B, C, D
        A = np.array([[0, 0, 0, 1, 0, 0],
                    [0, 0, 0, 0, 1, 0],
                    [0, 0, 0, 0, 0, 1],
                    [0, 0, 0, -kvx/m, 0, 0],
                    [0, 0, 0, 0, -kvy/m, 0],
                    [0, 0, 0, 0, 0, -kvz/m]])

        B = np.array([[0, 0, 0],
                    [0, 0, 0],
                    [0, 0, 0],
                    [self.g, 0, 0],
                    [0, self.g, 0],
                    [0, 0, 1/m]])
        
        C = np.array([[1, 0, 0, 0, 0, 0],
                      [0, 1, 0, 0, 0, 0],
                      [0, 0, 1, 0, 0, 0],
                      [0, 0, 0, 1, 0, 0],
                      [0, 0, 0, 0, 1, 0],
                      [0, 0, 0, 0, 0, 1]])

        D = np.zeros((C.shape[0], B.shape[1]))
        
        # FeedForward
        A_ffw = np.vstack((np.hstack((A, B)), np.hstack((C, D))))
        # Solving for Nx and Nu using least squares
        Nx_Nu = np.dot(np.linalg.pinv(A_ffw), np.vstack([np.zeros((6, 6)), np.eye(6)]))
        self.Nx = Nx_Nu[:6, :]
        self.Nu = Nx_Nu[6:, :]

        Q = np.array([[1, 0, 0, 0, 0, 0],
                    [0, 1, 0, 0, 0, 0],
                    [0, 0, 10 , 0, 0, 0],
                    [0, 0, 0, 1, 0, 0],
                    [0, 0, 0, 0, 1, 0],
                    [0, 0, 0, 0, 0, 1]])

        R = np.array([[10, 0, 0],
                    [0, 10, 0],
                    [0, 0, 0.1]])
        
        self.K, _, _ = control.lqr(A, B, Q, R)

    def compute_u(self, states, desired_states):
        # self.K = [[4.912865978705721, -0.000000000000001, -0.000000000000015, 1.248088498408166, 0.000000000000000, -0.000000000000000],
        #         [-0.000000000000001, 4.925415440833989, -0.000000000000004, -0.000000000000000, 1.220013553270824, -0.000000000000000],
        #         [0.000000000000001, -0.000000000000004, 4.068026001655915, 0.000000000000000, -0.000000000000000, 0.892025454665380]]
        
        u = self.Nu @ np.array(desired_states) - self.K @ (np.array(states) - self.Nx @ np.array(desired_states))

        # Normalizing angles and conversion to degrees
        u_theta = self.normalize_angle(u[0])/self.ang_max_rad
        u_phi   = self.normalize_angle(u[1])/self.ang_max_rad

        theta = np.sign(u_theta)*self.ang_max if np.abs(u_theta) > 1.0 else u_theta*self.ang_max
        phi   = np.sign(u_phi)*self.ang_max   if np.abs(u_phi)   > 1.0 else u_phi*self.ang_max

        # transforming thrust
        a_max = 15
        conversao_digital = 60000/a_max

        thrust = (u[2] + self.g) * conversao_digital
        thrust = min(max(thrust,11000),60000)

        return theta, phi, thrust
    
    def normalize_angle(self, angle):
        # Normalize the angle to be within the range [-π, π]
        angle = angle % (2 * np.pi)  # Normalize to range [0, 2π)
        if angle > np.pi:
            angle -= 2 * np.pi  # Adjust to [-π, π) if angle is greater than π
        elif angle < -np.pi:
            angle += 2 * np.pi  # Adjust to [-π, π) if angle is less than -π
        return angle