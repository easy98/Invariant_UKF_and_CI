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
        self.rot_mat = self.rotation_matrix(roll=self.roll, pitch=self.pitch, yaw=self.yaw)
        self.imu_angular_velocity_g = self.calculate_angular_velocity() 
        self.imu_angular_velocity_l = np.zeros(self.imu_angular_velocity_g.shape)
        self.imu_linear_acceleration_g = self.calculate_linear_acceleration() 
        self.imu_linear_acceleration_l = np.zeros(self.imu_linear_acceleration_g.shape)
        
        # self.add_noise()
        self.imu_global_to_local()

    def generate_trajectory(self):

        time = np.linspace(0, 10, self.total_steps)  # Total time of 10 seconds
        t1 = time * 4 * 2 * np.pi / 10     # 4 loops, adjust angle over time
        r1 = np.linspace(2, 6, self.total_steps)      # Radius from 2 to 6

        self.x = r1 * np.cos(t1)
        self.y = r1 * np.sin(t1)
        self.z = np.linspace(0, 8, self.total_steps)  # Z from 0 to 8

        self.yaw = t1 % (2 * np.pi)

        self.roll = np.linspace(0, np.pi/4, self.total_steps)
        # self.pitch[400:] = np.linspace(np.pi/4, 0, 600)

    def calculate_angular_velocity(self):
        dr = np.diff(self.roll)/self.dt
        dp = np.diff(self.pitch)/self.dt
        dy = np.diff(self.yaw)/self.dt

        dr = np.append(dr, dr[-1])
        dp = np.append(dp, dp[-1])
        dy = np.append(dy, dy[-1])

        wx = dr - np.sin(self.pitch)*dy
        wy = np.cos(self.roll)*dp + np.sin(self.roll)*np.cos(self.pitch)*dy
        wz = -np.sin(self.roll)*dp + np.cos(self.roll)*np.cos(self.pitch)*dy

        angular_velocity_matrix = np.vstack([wx, wy, wz])
        return angular_velocity_matrix

    def calculate_linear_acceleration(self):
        velocity = np.gradient(self.position, axis=1, edge_order=2) / self.dt
        # print(velocity)
        acceleration = np.gradient(velocity, axis=1, edge_order=2) / self.dt
        acceleration[2,:] -= 9.81
        return acceleration
    
    def rotation_matrix(self, roll, pitch, yaw):
        R_ls = []
        for i in range(self.total_steps):
            Rx = np.array([[1, 0, 0],
                            [0, np.cos(self.roll[i]), -np.sin(self.roll[i])],
                            [0, np.sin(self.roll[i]), np.cos(self.roll[i])]])

            Ry = np.array([[np.cos(self.pitch[i]), 0, np.sin(self.pitch[i])],
                            [0, 1, 0],
                            [-np.sin(self.pitch[i]), 0, np.cos(self.pitch[i])]])

            Rz = np.array([[np.cos(self.yaw[i]), -np.sin(self.yaw[i]), 0],
                            [np.sin(self.yaw[i]), np.cos(self.yaw[i]), 0],
                            [0, 0, 1]])
            R_ls.append(Rz @ Ry @ Rx)
        return np.array(R_ls)
    
    def imu_global_to_local(self):
        for i in range (self.total_steps):
            self.imu_angular_velocity_l[:,i] = self.rot_mat[i,:,:].T @ self.imu_angular_velocity_g[:,i]
            self.imu_linear_acceleration_l[:,i] = self.rot_mat[i,:,:].T @ self.imu_linear_acceleration_g[:,i]
            
    def add_noise(self):
        if self.noise_is_gaussian:
            self.imu_angular_velocity_g += np.random.normal(0, self.std_dev, self.imu_angular_velocity_g.shape)
            self.imu_linear_acceleration_g += np.random.normal(0, self.std_dev, self.imu_linear_acceleration_g.shape)
        else:
            self.imu_angular_velocity_g += np.random.uniform(-self.std_dev, self.std_dev, self.imu_angular_velocity_g.shape)
            self.imu_linear_acceleration_g += np.random.uniform(-self.std_dev, self.std_dev, self.imu_linear_acceleration_g.shape)

    
    
# Example usage
trajectory = Trajectory()
