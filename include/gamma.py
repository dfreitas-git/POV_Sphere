
# Program to generate a gamma table for a given value of gamma.  
# Run for different gamma values and store the tables in gamma.h

import math
gamma = 2.4
for i in range(256):
    v = int(pow(i / 255.0, gamma) * 255 + 0.5)
    print(f"{v},", end="")

