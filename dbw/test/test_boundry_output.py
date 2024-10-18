# tests that each set of inputs results in outputs in the correct range

import numpy as np

lin_vel_step_size = 1/128
lin_vel_min       = -250  # km/hr
lin_vel_eff_max   =  40   # km/h

ang_vel_step_size = 1/1024
ang_vel_min       = -31.25 # rad/s
ang_vel_eff_max   =  31.25 # rad/s

lin_percent_range  = 100.0
ang_percent_range  = 200.0
ang_percent_offset = 100.0

lin_offset    = abs(lin_vel_min / lin_vel_step_size)
lin_spd_range = lin_vel_eff_max / lin_vel_step_size

ang_offset    = abs(ang_vel_min / ang_vel_step_size)
ang_spd_range = (ang_vel_eff_max - ang_vel_min) / ang_vel_step_size

throttle_cmd_new = []
steering_cmd_new = []

for i in range(11):
    throttle_percentage = 150 * i/10
    steering_percentage = 200 * i/10 - 150

    gear = i%3

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

    if gear == 1:
        print('gear: forward')
        throttle_cmd = np.uint16(lin_spd_range *  throttle_percentage / lin_percent_range + lin_offset)
    elif gear == 2:
        print('gear: backward')
        throttle_cmd = np.uint16(lin_spd_range * -throttle_percentage / lin_percent_range + lin_offset)
    else:
        print('gear: neutral')
        throttle_cmd = np.uint16(lin_offset)
        
    throttle_cmd_new.append(throttle_cmd)
    
    steering_cmd = np.uint16(ang_spd_range * (steering_percentage + ang_percent_offset) / ang_percent_range)
    steering_cmd_new.append(steering_cmd)

print(throttle_cmd_new)
print(steering_cmd_new)