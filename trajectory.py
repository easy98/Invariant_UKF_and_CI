import numpy as np
import matplotlib.pyplot as plt


class Trajectory:
    def __init__(self, total_steps=1000, axis_length=0.5, dt = .01, noise_is_gaussian = True, std_dev = .02):
        self.total_steps = total_steps
        self.axis_length = axis_length
        self.dt = dt
        self.noise_is_gaussian = noise_is_gaussian
        self.std_dev = std_dev 
        
        self.x = np.zeros(self.total_steps)
        self.y = np.zeros(self.total_steps)
        self.z = np.zeros(self.total_steps)
        self.roll = np.zeros(self.total_steps)
        self.pitch = np.zeros(self.total_steps)
        self.yaw = np.zeros(self.total_steps)

        self.generate_trajectory()
        self.position = np.array([self.x, self.y, self.z])
        self.orientation = np.array([self.roll, self.pitch, self.yaw])
        self.R = self.rotation_matrix(self.trajectory.roll, self.trajectory.pitch, self.trajectory.yaw)
        self.imu_angular_velocity = self.calculate_linear_acceleration() @ self.R # global to local
        self.imu_linear_acceleration = self.calculate_angular_velocity() @ self.R # global to local
        self.add_noise()

    def generate_trajectory(self):
        # First 400 steps: Spiral
        t1 = np.linspace(0, 4 * 2 * np.pi, 400)
        self.z[:400] = np.floor(t1 / (2 * np.pi))
        r1 = 0.5 * self.z[:400]
        self.x[:400] = r1 * np.cos(t1)
        self.y[:400] = r1 * np.sin(t1)
        self.yaw[:400] = t1 % (2*np.pi)

        # Next 600 steps: infinity shape at constant height z = 8
        t2 = np.linspace(0, 12 * np.pi, 600)
        self.x[400:] = 3 * np.sin(t2)
        self.y[400:] = 3 * np.sin(t2) * np.cos(t2)
        self.z[400:] = 6
        self.yaw[400:] = t2 % (2*np.pi)
        
        self.roll[400:] = np.linspace(0, 2 * np.pi, 600)
        self.pitch[400:] = np.linspace(2 * np.pi, 0, 600)

    def calculate_angular_velocity(self):
        omega_roll = np.gradient(self.roll, self.dt)
        omega_pitch = np.gradient(self.pitch, self.dt)
        omega_yaw = np.gradient(self.yaw, self.dt)
        return np.array([omega_roll, omega_pitch, omega_yaw])

    def calculate_linear_acceleration(self):
        velocity = np.gradient(self.position, axis=1, edge_order=2) / self.dt
        acceleration = np.gradient(velocity, axis=1, edge_order=2) / self.dt
        acceleration[2] -= 9.81
        return acceleration
    
    def add_noise(self):
        if self.noise_is_gaussian:
            self.imu_linear_acceleration += np.random.normal(0, self.std_dev, self.imu_linear_acceleration.shape)
            self.imu_angular_velocity += np.random.normal(0, self.std_dev, self.imu_angular_velocity.shape)
        else:
            self.imu_linear_acceleration += np.random.uniform(-self.std_dev, self.std_dev, self.imu_linear_acceleration.shape)
            self.imu_angular_velocity += np.random.uniform(-self.std_dev, self.std_dev, self.imu_angular_velocity.shape)

    def rotation_matrix(self, roll, pitch, yaw):
        Rx = np.array([[np.cos(roll), -np.sin(roll), 0], [np.sin(roll), np.cos(roll), 0], [0, 0, 1]])
        Ry = np.array([[np.cos(pitch), -np.sin(pitch), 0], [np.sin(pitch), np.cos(pitch), 0], [0, 0, 1]])
        Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])
        return Rx @ Ry @ Rz
    
# Example usage
trajectory = Trajectory()
print(trajectory.imu_angular_velocity.shape)
