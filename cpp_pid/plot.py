import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

filename = '/home/auvsl/ros2_ws/2024-11-01_14-04-31'
cols = ['x','y', 'theta', 'input', 'output']
data = pd.read_csv(filename,names=cols,header=None)

plt.figure()
plt.plot(data['x'].values,data['y'].values)
plt.xlim(0,200)
plt.ylim(-100,0)
plt.xlabel('X Position (m)')
plt.ylabel('Y Position (m)')
plt.title('XY Position of Rgator When Manually Driving')

plt.figure()
plt.plot(data['theta'].values)
plt.title('VectorNav Yaw over Iterations')
plt.xlabel('Iterations')
plt.ylabel('VectorNav Yaw (rad)')

plt.figure()
plt.plot(data['input'].values)
plt.title('Input over Iterations')
plt.xlabel('Iterations')
plt.ylabel('Distance Line (m)')

plt.figure()
plt.plot(data['output'].values)
plt.title('Angular Velocity over Iterations')
plt.xlabel('Iterations')
plt.ylabel('Angular Velocity (rad/s)')

plt.show()