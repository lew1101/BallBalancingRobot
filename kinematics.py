from math import sqrt, asin

Vec3f = tuple[float, float, float]

SQRT3OVER2 = sqrt(3) / 2


def solveAngles(planeNormal: Vec3f, h: float, x: float, l1: float, l2: float, l3: float) -> Vec3f:

    def _solveAngle(planeNormal: Vec3f, h: float, x: float, l1: float, l2: float,
                    l3: float) -> float:
        a, _, c = planeNormal

        s1 = l3 / sqrt(a * a + c * c)
        xc = c * s1
        zc = h - a * s1

        xb = x + l1
        zb = zc - sqrt(-(xb + l2 - xc) * (xb - l2 - xc))

        return asin(zb / l1)

    a, b, c = planeNormal

    angle1 = _solveAngle(planeNormal, h, x, l1, l2, l3)

    rotatedNormal2 = (-a * 0.5 - b * SQRT3OVER2, -a * 0.5 + b * SQRT3OVER2, c)
    angle2 = _solveAngle(rotatedNormal2, h, x, l1, l2, l3)

    rotatedNormal3 = (-a * 0.5 + b * SQRT3OVER2, -a * 0.5 - b * SQRT3OVER2, c)
    angle3 = _solveAngle(rotatedNormal3, h, x, l1, l2, l3)

    return (angle1, angle2, angle3)
