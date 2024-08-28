import numpy as np
import plotly.graph_objects as go

class ObstacleAvoidance:
    def __init__(self, n, a, b, k, lambda_):
        if n == 0 or a == 0 or b == 0 or k == 0:
            raise ValueError("The obstacle detection constants must be declared")
        self.n = n
        self.a = a
        self.b = b
        self.k = k
        self.lambda_ = lambda_
        self.J = np.zeros((2, 1))  # Ensure J is properly sized

    def calculate_V(self, x_diff, y_diff):
        return np.exp(-np.power(x_diff, self.n) / self.a) * np.exp(-np.power(y_diff, self.n) / self.b)

    def j_ob(self, v, x_diff, y_diff):
        result = np.zeros(2)
        result[0] = -v * self.n * np.power(x_diff, self.n - 1) / self.a
        result[1] = -v * self.n * np.power(y_diff, self.n - 1) / self.b
        return result

    def obstacle_avoidance(self, robot_point, obstacle_point):
        x_diff = robot_point[0] - obstacle_point[0]
        y_diff = robot_point[1] - obstacle_point[1]

        v = self.calculate_V(x_diff, y_diff)

        J_ob = self.j_ob(v, x_diff, y_diff)
        self.J = J_ob.reshape(2, 1)
        v_ref = self.k * (-v)

        lambda_Job = self.lambda_ * np.eye(2)

        J_ob_T_J_ob = np.outer(J_ob, J_ob) + lambda_Job
        pseudo_inv_term = J_ob @ np.linalg.inv(J_ob_T_J_ob) * v_ref

        x_dot = pseudo_inv_term[0]
        y_dot = pseudo_inv_term[1]

        return x_dot, y_dot, v

    def get_J(self):
        return self.J

# Instantiate the ObstacleAvoidance class with example constants
obstacle_avoidance_system = ObstacleAvoidance(n=2, a=0.05, b=0.05, k=10.103757784844165, lambda_=100)

# Robot point at (0, 0)
robot_point = (0, 0)

# Generate a grid of obstacle points
x = np.linspace(0.01, 1, 500)
y = np.linspace(0.01, 1, 500)
X, Y = np.meshgrid(x, y)
U = np.zeros_like(X)
V = np.zeros_like(Y)
Z = np.zeros_like(X)

# Calculate velocity vectors and potential values for the grid
for i in range(X.shape[0]):
    for j in range(X.shape[1]):
        U[i, j], V[i, j], Z[i, j] = obstacle_avoidance_system.obstacle_avoidance(robot_point, (X[i, j], Y[i, j]))

# Create an interactive 3D plot using plotly
fig = go.Figure(data=[go.Surface(z=Z, x=X, y=Y, colorscale='Viridis')])

fig.update_layout(
    title='3D Plot of Obstacle Avoidance Potential Values',
    scene=dict(
        xaxis_title='X axis',
        yaxis_title='Y axis',
        zaxis_title='Potential Value'
    )
)

# Display the plot in the Jupyter Notebook
fig.show()