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
        self.z[:400] = np.floor(t1 / (2 * np.pi))
        r1 = 0.5 * self.z[:400]
        self.x[:400] = r1 * np.cos(t1)
        self.y[:400] = r1 * np.sin(t1)
        self.yaw[:400] = t1 % (2*np.pi)

        # Next 600 steps: Infinity shape at constant height z = 8
        t2 = np.linspace(0, 12 * np.pi, 600)
        self.x[400:] = 3 * np.sin(t2)
        self.y[400:] = 3 * np.sin(t2) * np.cos(t2)
        self.z[400:] = 6
        self.yaw[400:] = t2 % (2*np.pi)
        
        self.roll[400:] = np.linspace(0, 2 * np.pi, 600)
        self.pitch[400:] = np.linspace(2 * np.pi, 0, 600)
        return self.x, self.y, self.z, self.roll, self.pitch, self.yaw

traj = Trajectory()
print(traj.generate_trajectory())