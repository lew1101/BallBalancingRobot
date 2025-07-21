from math import sqrt, asin, degrees

Vec3f = tuple[float, float, float]

SQRT3OVER2 = sqrt(3) / 2


def _solveArmPosition(planeNormal: Vec3f, h: float, x: float, l1: float, l2: float, l3: float):
    a, _, c = planeNormal

    s1 = l3 / sqrt(a * a + c * c)
    xc = c * s1
    zc = h - a * s1

    s2 = x * x - 2 * x * xc + xc * xc + zc * zc
    s3 = l1 * l1 + l2 * l2
    s4 = s2 - s3

    arg = -(s4 - 2 * l1 * l2) * (s4 + 2 * l1 * l2)

    if arg < 0:
        raise ValueError(f"Cannot take square root of {arg}")

    s5 = sqrt(arg)
    s6 = l1**2 - l2**2
    s7 = s6 + xc**2

    xb = (x * x * x + x * zc * zc - s6 * x + s5 * zc + xc *
          (-x * x + zc * zc + s7) - x * xc * xc) / (2 * s2)
    zb = (s5 * x - s5 * xc + zc * zc * zc + zc * (x * x - 2 * xc * x + s7)) / (2 * s2)

    return (x, 0.0, 0.0), (xb, 0.0, zb), (xc, 0.0, zc)

def _solveArmHeight(planeNormal: Vec3f, h: float, x: float, l1: float, l2: float, l3: float):
    a, _, c = planeNormal

    s1 = l3 / sqrt(a * a + c * c)
    xc = c * s1
    zc = h - a * s1

    s2 = x * x - 2 * x * xc + xc * xc + zc * zc
    s3 = l1 * l1 + l2 * l2
    s4 = s2 - s3

    arg = -(s4 - 2 * l1 * l2) * (s4 + 2 * l1 * l2)

    if arg < 0:
        raise ValueError(f"Cannot take square root of {arg}")

    s5 = sqrt(arg)
    s6 = l1**2 - l2**2
    s7 = s6 + xc**2

    return (s5 * x - s5 * xc + zc * zc * zc + zc * (x * x - 2 * xc * x + s7)) / (2 * s2)


def _solveAngle(planeNormal: Vec3f, h: float, x: float, l1: float, l2: float, l3: float) -> float:
    return degrees(asin(_solveArmHeight(planeNormal, h, x, l1, l2, l3) / l1))


def _rotateSystem(vec: Vec3f, motorIndex: int) -> Vec3f:
    """Rotates normal vector by +\- 120 degrees in the XY plane"""
    a, b, c = vec
    if motorIndex == 1:
        return (-0.5 * a - SQRT3OVER2 * b, -0.5 * b + SQRT3OVER2 * a, c)
    elif motorIndex == 2:
        return (-0.5 * a + SQRT3OVER2 * b, -0.5 * b - SQRT3OVER2 * a, c)
    return vec


def _unrotateSystem(vec: Vec3f, motorIndex: int) -> Vec3f:
    """Rotates normal vector by +\- 120 degrees in the XY plane"""
    a, b, c = vec
    if motorIndex == 1:
        return (-0.5 * a + SQRT3OVER2 * b, -0.5 * b - SQRT3OVER2 * a, c)
    elif motorIndex == 2:
        return (-0.5 * a - SQRT3OVER2 * b, -0.5 * b + SQRT3OVER2 * a, c)
    return vec


def solveArmPositions(planeNormal: Vec3f, h: float, x: float, l1: float, l2: float, l3: float):
    armPositions = []

    for motorIndex in range(3):
        rotatedNormal = _rotateSystem(planeNormal, motorIndex)
        pseudoArmPos = _solveArmPosition(rotatedNormal, h, x, l1, l2, l3)
        trueArmPos = tuple(_unrotateSystem(point, motorIndex) for point in pseudoArmPos)

        armPositions.append(trueArmPos)

    return tuple(armPositions)


def solveAngles(planeNormal: Vec3f, h: float, x: float, l1: float, l2: float, l3: float):
    return tuple(
        _solveAngle(_rotateSystem(planeNormal, motorIndex), h, x, l1, l2, l3)
        for motorIndex in range(3))
