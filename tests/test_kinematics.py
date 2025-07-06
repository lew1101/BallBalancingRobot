import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # type: ignore

from ..constants import *
from ..kinematics import solveAngles, solveArmPositions

L = 8


def test_kinematics():
    plt.ion()

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    line, = ax.plot([], [], [], lw=2)

    ax.plot([0, 2], [0, 0], [0, 0], color='red')  # X-axis
    ax.plot([0, 0], [0, 2], [0, 0], color='green')  # Y-axis
    ax.plot([0, 0], [0, 0], [0, 2], color='blue')  # Z-axis
    ax.text(2, 0, 0, 'x', color='red')
    ax.text(0, 2, 0, 'y', color='green')
    ax.text(0, 0, 2, 'z', color='blue')

    ax.set_xlim(-L, L)
    ax.set_ylim(-L, L)
    ax.set_zlim(0, 2 * L)

    ax.grid(False)
    ax.xaxis.pane.fill = False
    ax.yaxis.pane.fill = False
    ax.zaxis.pane.fill = False

    normal = np.array([1, 2, 7])

    xx, yy = np.meshgrid(np.linspace(-L, L, 10), np.linspace(-L, L, 10))

    a, b, c = normal
    zz = (-a * xx - b * yy) / c + H
    ax.plot_surface(xx, yy, zz, alpha=0.5)

    arms = solveArmPositions(normal, H, X, L1, L2, L3)
    for arm in arms:
        xs = [p[0] for p in arm]
        ys = [p[1] for p in arm]
        zs = [p[2] for p in arm]

        # Draw lines between joints
        ax.plot(xs, ys, zs, marker='o')

    ax.quiver(*(0, 0, H), *normal, normalize=False, color='orange')

    plt.ioff()
    plt.show()
