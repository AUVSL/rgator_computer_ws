# This test aims to check the merge bytes function which takes in a high byte and low byte and merges them.

def merge_2_bytes(high_byte, low_byte):
    high = high_byte*2**8 + 0x00FF
    low  =         0xFF00 + low_byte

    return high & low

for i in range(8):
    num1 = int(255/7*i)
    num2 = 255-num1

    print("high byte in hexadecimal: ", format(num1, 'x'))
    print("low byte in hexadecimal: ", format(num2, 'x'))

    merged = merge_2_bytes(num1, num2)

    print("merged number: ", format(merged, 'x'))
    print()