import numpy as np
class CI:
    def __init__(self, X_sensors, X_robot):
        self.X_s = X_sensors # neighboring sensors
        self.X_r = X_robot # target
        self.num_sensors = self.X_s.shape[0]
        self.state_size = self.X_s.shape[1:]
        self.P_s = self.cov_mat(X_sensors)
        self.weights = self.CI_weights(self.P_s)
        self.Z_int_r, self.P_int_r = self.CI_interm_pair(self.X_r, self.X_s)

    def cov_mat(self, X):
        cov_matrices = np.zeros((self.num_sensors, 3, 3))

        for i in range(self.num_sensors):
            matrix = X[i]
            mean_col = np.mean(matrix, axis=0)
            centered_matrix = matrix - mean_col
            cov_matrix = np.dot(centered_matrix.T, centered_matrix) / (centered_matrix.shape[0])
            cov_matrices[i] = cov_matrix

        return cov_matrices

    def CI_weights(self, P):
        weights = np.zeros(self.num_sensors)
        den = 0

        for i in range(self.num_sensors):
            cov = P[i]
            den += 1 / np.trace(cov)
        for i in range(self.num_sensors):
            cov = P[i]
            num = 1 / np.trace(cov)
            weights[i] = num / den
        return weights

    def CI_interm_pair(self, X_r, X_s):
        p = np.zeros(self.state_size)
        for i in range(self.num_sensors):
            p += self.weights[i] * np.linalg.pinv(self.P_s[i])
        P_int_r = np.linalg.pinv(p)
        
        z = np.zeros(self.state_size)
        for i in range(self.num_sensors):
            z += self.weights[i] * np.linalg.pinv(self.P_s[i]) * np.log(X_s[i] @ np.linalg.pinv(X_r))
        Z_int_r = P_int_r @ z
        
        return Z_int_r, P_int_r


