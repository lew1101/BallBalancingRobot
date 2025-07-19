import numpy as np
import matplotlib.pyplot as plt

from src.constants import *
from src.kinematics import solveArmPositions

L = 100


def test_kinematics():
    plt.ion()

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    line, = ax.plot([], [], [], lw=2)

    ax.plot([0, L], [0, 0], [0, 0], color='red')  # X-axis
    ax.plot([0, 0], [0, L], [0, 0], color='green')  # Y-axis
    ax.plot([0, 0], [0, 0], [0, L], color='blue')  # Z-axis
    ax.text(L, 0, 0, 'x', color='red')
    ax.text(0, L, 0, 'y', color='green')
    ax.text(0, 0, L, 'z', color='blue')

    ax.set_xlim(-L, L)
    ax.set_ylim(-L, L)
    ax.set_zlim(0, 2 * L)

    ax.grid(False)
    ax.xaxis.pane.fill = False
    ax.yaxis.pane.fill = False
    ax.zaxis.pane.fill = False

    normal = np.array([0, MAX_XY, NORMAL_Z])

    xx, yy = np.meshgrid(np.linspace(-L, L, 10), np.linspace(-L, L, 10))

    a, b, c = normal
    zz = (-a * xx - b * yy) / c + H
    ax.plot_surface(xx, yy, zz, alpha=0.5)

    arms = solveArmPositions(normal, H, X, L1, L2, L3)
    for arm in arms:
        xs, ys, zs = zip(*arm)

        # Draw lines between joints
        ax.plot(xs, ys, zs, marker='o')

    ax.quiver(*(0, 0, H), *normal, normalize=False, color='orange')

    plt.ioff()
    plt.show()

