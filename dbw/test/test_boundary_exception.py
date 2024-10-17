import numpy as np

# This test aims to demonstrate how the function handles throttle and steering percentages that are out of the acceptable range. It prints a message to indicate that an invalid value has been given.

throttle_cmd = []
steering_cmd_new = []

for i in range(12):
    throttle_percentage = 150/11 * i - 25
    steering_percentage = 300/11 * i - 150

    if throttle_percentage < 0:
        throttle_percentage = 0
        print("WARNING: throttle percentage below range")
    elif throttle_percentage > 100:
        throttle_percentage = 100
        print("WARNING: throttle percentage above range")

    if steering_percentage < -100:
        steering_percentage = -100
        print("WARNING: steering percentage below range")
    elif steering_percentage > 100:
        steering_percentage = 100
        print("WARNING: steering percentage above range")


    gear = i%3

    if gear == 1:
        print('gear: forward')
        throttle_cmd.append(np.uint8(250.0*(throttle_percentage+100)/200.))
    elif gear == 2:
        print('gear: backward')
        throttle_cmd.append(np.uint8(250.0*(100.0 - throttle_percentage)/200.))

    else:
        print('gear: neutral')
        throttle_cmd.append(np.uint8(250.0*(0+100)/200.))

    steering_cmd_new.append(np.uint8(250.0*(steering_percentage+100)/200.))

print(throttle_cmd)
print(steering_cmd_new)