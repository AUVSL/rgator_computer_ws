import numpy as np

# This test is aimed to demonstrate the ability of the function to handle various throttle and steering commands and to compare the old and new methods of type-casting.

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

print("Old throttle_cmd: ", throttle_cmd_old)
print("New throttle_cmd: ", throttle_cmd_new)
print("Old steering_cmd: ", steering_cmd_old)
print("New steering_cmd: ", steering_cmd_new)