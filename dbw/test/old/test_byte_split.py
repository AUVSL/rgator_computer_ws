# This test aims to check the byte split function which splits a given 2-byte (16-bit) value into 2 separate bytes (8 bits each).
import numpy as np


def set_2_byte_number(byte_list, number, index):
    """Sets a 2-byte number in a list of bytes.

    Args:
        byte_list (list): The list of bytes.
        number (int): The 2-byte number to set.
        index (int): The starting index in the list to set the number.
    """

    if number < 0 or number > 65535:
        raise ValueError("Number must be between 0 and 65535")

    byte_list[index]     = (number >> 8) & 0xFF # High byte
    byte_list[index + 1] =        number & 0xFF # Low byte

    print("High byte: ", "{0:b}".format(byte_list[1]), "{0:x}".format(byte_list[0]))
    print("Low byte: " , "{0:b}".format(byte_list[0]), "{0:x}".format(byte_list[1]))

for i in range(8):
    number = np.uint16(65535/7 * i)
    print("Number in hexadecimal: ", format(number, 'x'))
    print("Number in binary: "     , format(number, 'b'))
    index = 0
    byte_list = [0,0]
    set_2_byte_number(byte_list, number, index)