import numpy as np

x = np.outer(
    np.cos(np.linspace(0, 2 * np.pi, 100)), np.sin(np.linspace(0, 2 * np.pi, 100))
)
y = np.outer(
    np.sin(np.linspace(0, 2 * np.pi, 100)), np.sin(np.linspace(0, 2 * np.pi, 100))
)
z = np.outer(np.ones(100), np.cos(np.linspace(0, 2 * np.pi, 100)))

fig = plt.figure()
ax = fig.add_subplot(projection="3d")
ax.plot_surface(x, y, z)
plt.show()
