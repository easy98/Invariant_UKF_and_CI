import numpy as np

class SensorNode:
    def __init__(self, sensor_num, sensor_x, sensor_y, sensor_z = 10):
        self.num = sensor_num
        self.x = sensor_x
        self.y = sensor_y
        self.z = sensor_z
        self.p = np.asarray([self.x, self.y, self.z])
        self.size = .2
        
class Sensors:
    def __init__(self):
        sensor_network = []
        sx = [5, 5, -5, -5]
        sy = [5, -5, 5, -5]
        for i in range(4):
            sensor_node = SensorNode(i, sx[i], sy[i])
            sensor_network.append(sensor_node)
    
    def projection_matrix():
        pass
    
    def rotation_matrix():
        pass