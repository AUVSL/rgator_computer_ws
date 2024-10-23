# This test aims to check the byte split function which splits a given 2-byte (16-bit) value into 2 separate bytes (8 bits each).

for i in range(8):
    number = int(65535/7 * i)
    print("Number in hexadecimal: ", format(number, 'x'))
    print("Number in binary: ", format(number, 'b'))
    index = 0
    byte_list = [0,0]
    byte_list[index]     = number & 0xFF  # Low byte
    byte_list[index + 1] = (number >> 8) & 0xFF  # High byte

    print("High byte: ", "{0:b}".format(byte_list[1]), "{0:x}".format(byte_list[1]))
    print("Low byte: " , "{0:b}".format(byte_list[0]), "{0:x}".format(byte_list[0]))

    print()