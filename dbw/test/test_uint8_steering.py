import numpy as np

l1 = []
l2 = []
for i in range(12):
    steering_percentage = 200.0/11 * i    # testing values in a range from -100 to 100, including boundary values

    l1.append(np.uint8(int(hex(int(250.0*(steering_percentage+100)/200.)), 16)))    # previous calculation of code
    l2.append(np.uint8(250.0*(steering_percentage+100)/200.))

print(l1)
print(l2)

