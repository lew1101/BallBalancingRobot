import numpy as np
import matplotlib.pyplot as plt

from .constants import *
from .kinematics import solveArmPositions

L = 100.0

plt.ion()

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
line, = ax.plot([], [], [], lw=2)


def plotInit():
    ax.set_xlim(-L, L)
    ax.set_ylim(-L, L)
    ax.set_zlim(0, 2 * L)

    ax.plot([0, L], [0, 0], [0, 0], color='red')  # X-axis
    ax.plot([0, 0], [0, L], [0, 0], color='green')  # Y-axis
    ax.plot([0, 0], [0, 0], [0, L], color='blue')  # Z-axis

    ax.text(L, 0, 0, 'x', color='red')
    ax.text(0, L, 0, 'y', color='green')
    ax.text(0, 0, L, 'z', color='blue')

    ax.grid(False)
    ax.xaxis.pane.fill = False
    ax.yaxis.pane.fill = False
    ax.zaxis.pane.fill = False


def updatePlot(ballPos, planeNormal, arm1, arm2, arm3):
    xx, yy = np.meshgrid(np.linspace(-L, L, 10), np.linspace(-L, L, 10))

    a, b, c = planeNormal
    zz = (-a * xx - b * yy) / c + H
    ax.plot_surface(xx, yy, zz, alpha=0.5)

    xs, ys, zs = zip(arm1, arm2, arm3)

    # Draw lines between joints
    ax.plot(xs, ys, zs, marker='o')

    ax.quiver(*(0, 0, H), *planeNormal, normalize=False, color='orange')

    fig.canvas.draw()
    fig.canvas.flush_events()
