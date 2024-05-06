#!/usr/bin/env python3
import numpy as np
from numba import jit

class ObstacleAvoidance:
    def __init__(self, n=None, a=None, b=None, k=None):
        self.J = None
        if n==None or a==None or b==None or k==None:
            raise TypeError("The obstacle detection constants must be declared ")

        ## constants
        self.n = n
        self.a = a
        self.b = b
        self.k = k

    def calculate_V(self, x_diff, y_diff):
        return np.exp(-np.power((x_diff)/self.a, self.n))*np.exp(-np.power((y_diff)/self.b, self.n))

    def j_ob(self, v, x_diff, y_diff):
        return -v * self.n * np.array([(np.power((x_diff), self.n-1) / np.power(self.a, self.n)), np.power((y_diff), self.n-1) / np.power(self.b, self.n)])

    def obstacle_avoidance(self, robot_point, obstacle_points):
        x_dot, y_dot = 0, 0

        for obstacle_point in obstacle_points:
            x_diff = robot_point[0] - obstacle_point[0]
            y_diff = robot_point[1] - obstacle_point[1]

            v = self.calculate_V(x_diff, y_diff)

            J_ob = self.j_ob(v, x_diff, y_diff)
            self.J = J_ob
            v_ref = self.k * (-v)
            x_dot_aux, y_dot_aux = np.dot(np.linalg.pinv(J_ob.reshape(-1, 1)), v_ref)[0]
            x_dot+=x_dot_aux
            y_dot+=y_dot_aux


        return x_dot, y_dot

    def get_J(self):
        return self.J

if __name__ == '__main__':
    pass