import numpy as np

# This test is aimed to compare different methods of type-casting. The old method is printed first and the updated method is printed second.

l1 = []
l2 = []
for i in range(12):
    steering_percentage = 200.0/11 * i    # testing values in a range from -100 to 100, including boundary values

    l1.append(np.uint8(int(hex(int(250.0*(steering_percentage+100)/200.)), 16)))    # previous code
    l2.append(np.uint8(250.0*(steering_percentage+100)/200.))                       # updated code

print(l1)
print(l2)

