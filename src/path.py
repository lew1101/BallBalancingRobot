from math import pi, sin, cos

import numpy as np

Vec2f = tuple[float, float]


class Path:

    def __init__(self, path: list[Vec2f], *, loop: bool = False):
        self.path = path
        self.length = len(path)
        self.loop = loop

    def __iter__(self):
        index = 0
        while True:
            if index >= self.length:
                if self.loop:
                    index = 0
                else:
                    return  # raises StopIteration
            yield self.path[index]
            index += 1

    def __len__(self):
        return len(self.path)


def createSetPoint(point: Vec2f) -> Path:
    return Path([point], loop=True)


def createCircularPath(radius: float, n: int = 100, centre=(0.0, 0.0)) -> Path:
    cx, cy = centre
    
    theta = np.linspace(0, 2 * np.pi, 100)

    x = cx + radius * np.cos(theta)
    y = cy + radius * np.sin(theta)

    return Path(list(zip(x, y)), loop=True)


def pathFactory(pathtype: str, *args, **kwargs) -> Path:
    if pathtype == "setpoint":
        return createSetPoint(*args, **kwargs)
    elif pathtype == "circle":
        return createCircularPath(*args, **kwargs)
    elif pathtype == "custom":
        return Path(*args, **kwargs)
    else:
        raise ValueError(f"Unknown path type: '{pathtype}'")
