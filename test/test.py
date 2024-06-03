from trajectory import Trajectory
from scipy.linalg import expm, sinm, cosm
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import plotly.graph_objects as go
import numpy as np

total_steps = 1000
traj = Trajectory(total_steps = total_steps)

rot = traj.rot_mat # 1000,3,3
loc_w = traj.imu_angular_velocity_l # 3,1000
loc_a = traj.imu_linear_acceleration_l # 3,1000
est_x = np.zeros(total_steps)
est_y = np.zeros(total_steps)
est_z = np.zeros(total_steps)


est_x[0] = 2
est_y[0] = 0
est_z[0] = 0
est_R = []
est_R.append(rot[0])

# Measurement
dt = .01
# Velocity initialization
vx, vy, vz = 0.40004251, 5.02706154, 0.80008001

# Loop through each time step
for i in range(1, 1000):
    # Determine index for rotation matrix (assuming 10 times less samples for rotation matrices)
    rot_idx = i

    # Angular velocities (converted to global)
    w = loc_w[:, i]
    w_skew = np.array([[0, -w[2], w[1]],
                       [w[2], 0, -w[0]], [-w[1], w[0], 0]])
    est_R.append(est_R[-1]@expm(w_skew*dt))




    # Linear accelerations (converted to global, minus gravity effect assuming it is included)
    a = (est_R[-2])@(loc_a[:, i])+ np.array([0,0,9.81])

    # Update velocities
    vx += a[0] * dt
    vy += a[1] * dt
    vz += a[2] * dt
    # Update positions
    est_x[i] = est_x[i-1] + vx * dt + a[0]*dt**2/2
    est_y[i] = est_y[i-1] + vy * dt + a[1]*dt**2/2
    est_z[i] = est_z[i-1] + vz * dt + a[2]*dt**2/2

# Plotting the trajectory in 3D
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot(est_x, est_y, est_z)
ax.set_xlabel('X position (m)')
ax.set_ylabel('Y position (m)')
ax.set_zlabel('Z position (m)')
plt.title('3D Trajectory of IMU')
plt.show()
