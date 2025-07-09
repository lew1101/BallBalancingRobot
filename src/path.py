from math import pi, sin, cos

Vec2f = tuple[float, float]


class Path:

    def __init__(self, path: list[Vec2f], *, loop: bool = False):
        self.path = path
        self.index = 0
        self.length = len(path)
        self.initialPoint = path[0]
        self.loop = loop

    def hasNext(self) -> bool:
        return self.loop or self.index < self.length

    def next(self):
        while True:
            while self.index < self.length:
                yield self.path[self.index]
                self.index += 1

            if self.loop:
                self.index = 0
            else:
                return

    def __iter__(self):
        return self.path

    def __len__(self):
        return len(self.path)


def createSetPoint(point: Vec2f) -> Path:
    return Path([point], loop=True)


def createCircularPath(radius: float, n: int = 100, centre=(0.0, 0.0)) -> Path:
    path = []

    cx, cy = centre

    for i in range(n):
        angle = 2 * pi * i / n
        x = cx + radius * cos(angle)
        y = cy + radius * sin(angle)

        path.append((x, y))

    return Path(path, loop=True)


def pathFactory(pathtype: str, *args, **kwargs) -> Path:
    match pathtype:
        case "setpoint":
            return createSetPoint(*args, **kwargs)
        case "circle":
            return createCircularPath(*args, **kwargs)
        case _:
            raise ValueError(f"Unknown path type: '{pathtype}'")
