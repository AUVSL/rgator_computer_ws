# tests that each set of inputs results in outputs in the correct range

import numpy as np

lin_vel_min       = -250  # km/hr
lin_vel_step_size = 1/128
lin_spd_eff_min   = -40   # km/h
lin_spd_eff_max   =  40   # km/h
percent_offset    = 100.0
percent_range     = 200.0

output_offset = abs(lin_vel_min / lin_vel_step_size)
print(output_offset)
lin_spd_range = (lin_spd_eff_max - lin_spd_eff_min) / lin_vel_step_size

throttle_cmd_new = []
steering_cmd_new = []

for i in range(12):
    throttle_percentage = 100/11 * i
    steering_percentage = 200/11 * i - 100

    gear = i%3

    if gear == 1:
        print('gear: forward')
        throttle_cmd = np.uint16(lin_spd_range * ( throttle_percentage + percent_offset) / percent_range + output_offset)
        throttle_cmd_new.append(throttle_cmd)
    elif gear == 2:
        print('gear: backward')
        throttle_cmd = np.uint16(lin_spd_range * (-throttle_percentage + percent_offset) / percent_range + output_offset)
        throttle_cmd_new.append(throttle_cmd)
    else:
        print('gear: neutral')
        throttle_cmd = np.uint16(lin_spd_range * (                  0 + percent_offset) / percent_range + output_offset)
        throttle_cmd_new.append(throttle_cmd)

    steering_cmd     = np.uint16(lin_spd_range * (steering_percentage + percent_offset) / percent_range + output_offset)
    steering_cmd_new.append(steering_cmd)

print(throttle_cmd_new)
print(steering_cmd_new)