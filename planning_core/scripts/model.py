import numpy as np

a = np.array([[0, 0, -8, 0, -0.0027], [0, 0, -0.0218, 0, 1],
              [0, 0, 0, 2.66254, 0], [0, 0, 0, 0, 0], [0, 0, 0, 0, 0]])
I = np.identity(5)
dt = 0.05
ad = np.linalg.inv(I - dt * a / 2.0) @ (I + dt * a / 2.0)

print(ad)