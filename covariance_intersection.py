import numpy as np
class CI:
    def __init__(self, X_sensors, X_robot):
        self.X_s = X_sensors # neighboring sensors
        self.X_r = X_robot # target
        self.P_s = self.cov_mat(X_sensors)
        self.weights = self.CI_weights(self.P_s)


    def cov_mat(self, X):
        n_samples = X.shape[0]
        print(n_samples)
        cov_matrices = np.zeros((n_samples, 3, 3))

        for i in range(n_samples):
            matrix = X[i]
            mean_col = np.mean(matrix, axis=0)
            centered_matrix = matrix - mean_col
            cov_matrix = np.dot(centered_matrix.T, centered_matrix) / (centered_matrix.shape[0])
            cov_matrices[i] = cov_matrix

        return cov_matrices

    def CI_weights(self, P):
        weights = np.zeros(8)
        den = 0

        for i in range(8):
            cov = P[i]
            den += 1 / np.trace(cov)
        for i in range(8):
            cov = P[i]
            num = 1 / np.trace(cov)
            weights[i] = num / den
        return weights

    def CI_interm_pair(self, w, P, X_r, X_s):


