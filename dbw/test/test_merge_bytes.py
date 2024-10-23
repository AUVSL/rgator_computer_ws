# This test aims to check the merge bytes function which takes in a high byte and low byte and merges them.

for i in range(8):
    num1 = int(255/7*i)
    num2 = 255-num1

    print("num1 in hexadecimal: ", format(num1, 'x'))
    print("num2 in hexadecimal: ", format(num2, 'x'))

    print("num1 in binary: ", format(num1, 'b'))
    print("num2 in binary: ", format(num2, 'b'))

    num1_merge = num1*2**8 + 0xFF
    num2_merge = 0xFF00 + num2

    merged = num1_merge & num2_merge

    print("merged number: ", format(merged, 'x'))
    print()


def merge_bytes(high_byte, low_byte):
    high = high_byte*2**8 + 0xFF
    low = 0xFF00 + low_byte

    merged = high & low

    return merged