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
        return self.loop or self.index < self.length - 1

    def next(self):
        if self.index < self.length - 1:
            self.index += 1
            return self.path[self.index]
        elif self.loop:
            self.index = 0
            return self.initialPoint
        return None

    def __iter__(self):
        return self.path

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
    match pathtype:
        case "setpoint":
            return createSetPoint(*args, **kwargs)
        case "circle":
            return createCircularPath(*args, **kwargs)
        case "custom":
            return Path(*args, **kwargs)
        case _:
            raise ValueError(f"Unknown path type: '{pathtype}'")
