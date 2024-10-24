# tests that each set of inputs results in outputs in the correct range
import can
import numpy as np

propulsion   = can.Message(arbitration_id=0x18FFFF2A, data=[0x81, 0x7D, 0x00, 0x7D, 0x00, 0xFF, 0xFF, 0xFF], is_extended_id=True)

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

data_new = []

def set_2_bytes_number(byte_list, number, index):
        """Sets a 2-byte number in a list of bytes.

        Args:
            byte_list (list): The list of bytes.
            number (int): The 2-byte number to set.
            index (int): The starting index in the list to set the number.
        """

        if number < 0 or number > 65535:
            raise ValueError("Number must be between 0 and 65535")

        byte_list[index]     = number & 0xFF        # Low byte
        byte_list[index + 1] = (number >> 8) & 0xFF # High byte

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
        
    steering_cmd = np.uint16(ang_spd_range * (steering_percentage + ang_percent_offset) / ang_percent_range)
    
    # throttle
    set_2_bytes_number(propulsion.data, throttle_cmd, 1)
        
    # steering
    set_2_bytes_number(propulsion.data, steering_cmd, 3)
    
    data_new.append(propulsion.data)

print(data_new)