import matplotlib.pyplot as plt
import numpy as np

epsilon = 2


def hinge_loss(c):
    if c < 0:
        return 0
    elif 0 < c <= epsilon:
        return c**3
    elif epsilon < c:
        return 3*epsilon*c**2 - 3*epsilon**2*c + epsilon**3


x = np.linspace(-10, 10, 100)

plt.plot(x, [hinge_loss(ci) for ci in epsilon - x])
plt.show()
