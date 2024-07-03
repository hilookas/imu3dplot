import matplotlib.pyplot as plt
import numpy as np
import time

plt.ion()

fig = plt.figure()
ax = fig.add_subplot(projection='3d')

# Prepare arrays x, y, z
theta = np.linspace(-4 * np.pi, 4 * np.pi, 100)
z = np.linspace(-2, 2, 100)
r = z**2 + 1
x = r * np.sin(theta)
y = r * np.cos(theta)

ax.plot(x, y, z, label='parametric curve')
ax.legend()

while True:
    fig.canvas.draw()
    fig.canvas.flush_events()
    time.sleep(2)
