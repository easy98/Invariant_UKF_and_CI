import numpy as np
import matplotlib.pyplot as plt


class Trajectory:
    def __init__(self, total_steps=10000, axis_length=0.5, dt = .001, noise_is_gaussian = True, std_dev = .02):
        self.total_steps = total_steps
        self.axis_length = axis_length
        self.dt = dt
        self.noise_is_gaussian = noise_is_gaussian
        self.std_dev = std_dev 
        
        # self.waypoints1 = np.array([
        #     (4.245, 1.332, .312, .158, .123, .098),
        #     (4.374, 1.521, 2.432, .446, .379, .765),
        #     (3.987, 2.137, 3.576, .367, .459, 1.258),
        #     (5.023, 2.462, 4.184, .573, .224, 1.783),
        #     (5.483, 3.479, 4.940, .532, .415, 1.662),
        #     (6.327, 4.579, 5.872, .724, .471, 1.288),
        #     (6.589, 5.824, 7.012, .581, .347, .916),
        #     (7.036, 6.924, 7.318, .293, .328, .473)
        # ])

        # self.current_trajectory = self.create_trajectory(self.waypoints1,10,total_steps)
        # self.x = self.current_trajectory[:, 0]
        # self.y = self.current_trajectory[:, 1]
        # self.z = self.current_trajectory[:, 2]
        # self.roll = self.current_trajectory[:, 3]
        # self.pitch = self.current_trajectory[:, 4]
        # self.yaw = self.current_trajectory[:, 5]
        self.x = np.zeros(self.total_steps)
        self.y = np.zeros(self.total_steps)
        self.z = np.zeros(self.total_steps)
        self.roll = np.zeros(self.total_steps)
        self.pitch = np.zeros(self.total_steps)
        self.yaw = np.zeros(self.total_steps)
        
        self.generate_trajectory()
        
        self.position = np.array([self.x, self.y, self.z])
        self.orientation = np.array([self.roll, self.pitch, self.yaw])
        # self.R = self.rotation_matrix(roll=self.roll, pitch=self.pitch, yaw=self.yaw)
        self.imu_angular_velocity = self.calculate_linear_acceleration() #@ self.R # global to local
        self.imu_linear_acceleration = self.calculate_angular_velocity() #@ self.R # global to local
        self.add_noise()
    
    # def create_trajectory(self, waypoints, duration, frequency):
    #     # Calculate the number of points in the trajectory
    #     total_points = duration * frequency + 1  # +1 to include the endpoint
        
    #     times = np.linspace(0, duration, num=total_points)
        
    #     # Interpolate for each dimension independently
    #     self.x = np.interp(times, np.linspace(0, duration, num=len(waypoints)), waypoints[:, 0])
    #     self.y = np.interp(times, np.linspace(0, duration, num=len(waypoints)), waypoints[:, 1])
    #     self.z = np.interp(times, np.linspace(0, duration, num=len(waypoints)), waypoints[:, 2])
        
    #     self.roll = np.interp(times, np.linspace(0, duration, num=len(waypoints)), waypoints[:, 3])
    #     self.pitch = np.interp(times, np.linspace(0, duration, num=len(waypoints)), waypoints[:, 4])
    #     self.yaw = np.interp(times, np.linspace(0, duration, num=len(waypoints)), waypoints[:, 5])
    
    #     return np.column_stack((self.x, self.y, self.z, self.roll, self.pitch, self.yaw))

    def generate_trajectory(self):
        time = np.linspace(0, 10, 10000)  # Total time of 10 seconds
        t1 = time * 4 * 2 * np.pi / 10     # 4 loops, adjust angle over time
        r1 = np.linspace(2, 6, 10000)      # Radius from 2 to 6

        self.x = r1 * np.cos(t1)
        self.y = r1 * np.sin(t1)
        self.z = np.linspace(0, 8, 10000)  # Z from 0 to 8
        self.yaw = t1 % (2 * np.pi)  
        
        self.roll[4000:] = np.linspace(0, np.pi/4, 6000)
        self.pitch[4000:] = np.linspace(np.pi/4, 0, 6000)

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
    
    def rotation_matrix(self, roll, pitch, yaw):
        Rx = np.array([[1, 0, 0],
                    [0, np.cos(roll), -np.sin(roll)],
                    [0, np.sin(roll), np.cos(roll)]])
        
        Ry = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                    [0, 1, 0],
                    [-np.sin(pitch), 0, np.cos(pitch)]])
        
        Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                    [np.sin(yaw), np.cos(yaw), 0],
                    [0, 0, 1]])
        
        return Rz @ Ry @ Rx
    
    def add_noise(self):
        if self.noise_is_gaussian:
            self.imu_linear_acceleration += np.random.normal(0, self.std_dev, self.imu_linear_acceleration.shape)
            self.imu_angular_velocity += np.random.normal(0, self.std_dev, self.imu_angular_velocity.shape)
        else:
            self.imu_linear_acceleration += np.random.uniform(-self.std_dev, self.std_dev, self.imu_linear_acceleration.shape)
            self.imu_angular_velocity += np.random.uniform(-self.std_dev, self.std_dev, self.imu_angular_velocity.shape)

    
    
# Example usage
trajectory = Trajectory()
print(trajectory.imu_angular_velocity.shape)
