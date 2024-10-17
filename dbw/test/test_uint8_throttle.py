import numpy as np

throttle_cmd_old = []
throttle_cmd_new = []
steering_cmd_old = []
steering_cmd_new = []

for i in range(12):
    throttle_percentage = 100/11 * i
    steering_percentage = 200/11 * i - 100

    gear = i%3

    if gear == 1:
        print('gear: forward')
        throttle_cmd_old.append(np.uint8(int(hex(int(250.0*(throttle_percentage+100)/200.)), 16)))
        throttle_cmd_new.append(np.uint8(250.0*(throttle_percentage+100)/200.))
    elif gear == 2:
        print('gear: backward')
        throttle_cmd_old.append(np.uint8(int(hex(int(250.0*(100.0 - throttle_percentage)/200.)), 16)))
        throttle_cmd_new.append(np.uint8(250.0*(100.0 - throttle_percentage)/200.))

    else:
        print('gear: neutral')
        throttle_cmd_old.append(np.uint8(int(hex(int(250.0*(0+100)/200.)), 16)))
        throttle_cmd_new.append(np.uint8(250.0*(0+100)/200.))

    steering_cmd_old.append(np.uint8(int(hex(int(250.0*(steering_percentage+100)/200.)), 16)))
    steering_cmd_new.append(np.uint8(250.0*(steering_percentage+100)/200.))

print(throttle_cmd_old)
print(throttle_cmd_new)
print(steering_cmd_old)
print(steering_cmd_new)