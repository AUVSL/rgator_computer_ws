import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

filename = '/home/auvsl/ros2_ws/2024-10-09_19-26-50'
cols = ['x','y', 'theta', 'input1', 'input2', 'input3', 'input4', 'linear', 'angular']
data = pd.read_csv(filename,names=cols,header=None)

plt.figure()
plt.plot(np.array([-9.60897, -0.608969, 8.39103, 15.391]),np.array([-2.95891, -2.95891, -2.95891, -2.95891]), 'o')
plt.plot(data['x'].values,data['y'].values)
plt.xlim(-50,50)
plt.ylim(-50,50)
plt.xlabel('X Position (m)')
plt.ylabel('Y Position (m)')
plt.title('XY Position of Rgator When Manually Driving')

plt.figure()
plt.plot(data['theta'].values)
plt.title('VectorNav Yaw over Iterations')
plt.xlabel('Iterations')
plt.ylabel('VectorNav Yaw (rad)')

# plt.figure()
# plt.plot(data['input1'].values)
# plt.title('Distance Line over Iterations')
# plt.xlabel('Iterations')
# plt.ylabel('Distance Line (m)')

# plt.figure()
# plt.plot(data['input2'].values)
# plt.title('Theta Near over Iterations')
# plt.xlabel('Iterations')
# plt.ylabel('Theta Near (rad)')

# plt.figure()
# plt.plot(data['input3'].values)
# plt.title('Theta Far over Iterations')
# plt.xlabel('Iterations')
# plt.ylabel('Theta Far (rad)')

# plt.figure()
# plt.plot(data['input4'].values)
# plt.title('Distance Target over Iterations')
# plt.xlabel('Iterations')
# plt.ylabel('Distance Target (m)')

# plt.figure()
# plt.plot(data['linear'].values)
# plt.title('Linear Speed over Iterations')
# plt.xlabel('Iterations')
# plt.ylabel('Linear Speed (m/s)')

# plt.figure()
# plt.plot(data['angular'].values)
# plt.title('Agnular Velocity over Iterations')
# plt.xlabel('Iterations')
# plt.ylabel('Angular Velocity (rad/s)')

plt.show()