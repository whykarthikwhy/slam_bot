import numpy as np
a = np.array([5,6,7])


def plusone(x):
    x += 1
    print(x)
print(a)
plusone(a.copy())
print(a)