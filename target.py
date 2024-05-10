import numpy as np
from trajectory import Trajectory
from sensor import Sensors

class Target:
    def __init__(self):
        self.trajectory = Trajectory()
        self.sensors = Sensors()
        self.R = self.rotation_matrix(self.trajectory.roll, self.trajectory.pitch, self.trajectory.yaw)
        self.w, self.a = self.imu_propagation()
    
    def imu_propagation():
        pass
    
    def rotation_matrix(self, roll, pitch, yaw):
        Rx = np.array([[np.cos(roll), -np.sin(roll), 0], [np.sin(roll), np.cos(roll), 0], [0, 0, 1]])
        Ry = np.array([[np.cos(pitch), -np.sin(pitch), 0], [np.sin(pitch), np.cos(pitch), 0], [0, 0, 1]])
        Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])
        return Rx @ Ry @ Rz
    
    
        