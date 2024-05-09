import numpy as np
import matplotlib.pyplot as plt


class Trajectory:
    def __init__(self, total_steps=1000, axis_length=0.5):
        self.total_steps = total_steps
        self.axis_length = axis_length
        self.x = np.zeros(self.total_steps)
        self.y = np.zeros(self.total_steps)
        self.z = np.zeros(self.total_steps)
        self.roll = np.zeros(self.total_steps)
        self.pitch = np.zeros(self.total_steps)
        self.yaw = np.zeros(self.total_steps)
        self.generate_trajectory()

    def generate_trajectory(self):
        # First 400 steps: Spiral
        t1 = np.linspace(0, 4 * 2 * np.pi, 400)
        r1 = 0.5 * np.floor(t1 / (2 * np.pi))
        self.x[:400] = 5 + r1 * np.cos(t1)
        self.y[:400] = 5 + r1 * np.sin(t1)
        self.z[:400] = np.floor(t1 / (2 * np.pi))

        # Next 600 steps: Infinity shape at constant height z = 8
        t2 = np.linspace(0, 12 * np.pi, 600)
        self.x[400:] = 5 + 3 * np.sin(t2)
        self.y[400:] = 5 + 3 * np.sin(t2) * np.cos(t2)
        self.z[400:] = 8

    def rotation_matrix(self, roll, pitch, yaw):
        R_x = np.array([[1, 0, 0],
                        [0, np.cos(roll), -np.sin(roll)],
                        [0, np.sin(roll), np.cos(roll)]])
        
        R_y = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                        [0, 1, 0],
                        [-np.sin(pitch), 0, np.cos(pitch)]])
        
        R_z = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                        [np.sin(yaw), np.cos(yaw), 0],
                        [0, 0, 1]])

        return np.dot(R_z, np.dot(R_y, R_x))

    def plot_trajectory(self):
        fig = plt.figure(figsize=(12, 10))
        ax = fig.add_subplot(111, projection='3d')
        ax.plot(self.x, self.y, self.z, label='Trajectory')

        colors = ['red', 'green', 'blue']  # Colors for x, y, z axes
        # Adding local coordinate axes every 100 steps
        for i in range(0, self.total_steps, 100):
            R = self.rotation_matrix(self.roll[i], self.pitch[i], self.yaw[i])
            for j in range(3):
                ax.quiver(self.x[i], self.y[i], self.z[i], R[j, 0], R[j, 1], R[j, 2], color=colors[j])

        ax.set_xlabel('X Coordinate')
        ax.set_ylabel('Y Coordinate')
        ax.set_zlabel('Z Coordinate')
        ax.set_title('3D Trajectory with Local Coordinate Axes')
        ax.legend()

        plt.show()
