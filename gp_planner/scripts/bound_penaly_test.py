import numpy as np
import matplotlib.pyplot as plt


class Penalty(object):
    def __init__(self, limit, eps):
        self.limit1 = limit
        self.limit2 = limit + eps
        self.a2 = 3 * eps
        self.b2 = 3 * eps**2 - 2 * self.a2 * self.limit2
        self.c2 = eps**3 - self.a2 * self.limit2**2 - self.b2 * self.limit2

        self.a1 = 3 * eps
        self.b1 = - 3 * eps**2 + 2 * self.a1 * self.limit2
        self.c1 = eps**3 - self.a1 * self.limit2**2 + self.b1 * self.limit2

    def penalty(self, val):
        if abs(val) < self.limit1:
            return [0.0, 0.0]
        elif self.limit1 <= val and val <= self.limit2:
            error = val - self.limit1
            return [error**3, 3*error**2]
        elif self.limit2 <= val:
            return [self.a2 * val**2 + self.b2 * val + self.c2]
        elif -self.limit2 < val and val <= -self.limit1:
            error = -self.limit1 - val
            return [error ** 3, -3 * error**2]
        else:
            return [self.a1 * val**2 + self.b1 * val + self.c1, 2*self.a1 * val + self.b1]


p = Penalty(0.35, 0.35 * 2)
t = np.linspace(-0.5, 0.5, 500)
x = np.array([p.penalty(i)[0] for i in t])
dx = np.array([p.penalty(i)[1] for i in t])

plt.subplot(2,1,1)
plt.plot(t, x)
plt.subplot(2,1,2)
plt.plot(t, dx)
plt.show()
