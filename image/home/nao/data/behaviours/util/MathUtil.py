import math

EPSILON = 0.00001


def angleDiff(angle0, angle1):
    _angle_diff = math.fabs(angle0 - angle1)
    return min(_angle_diff, math.fabs(_angle_diff - 2.0 * math.pi))


def angleSignedDiff(target, current):
    diff = target - current
    return reduce( # noqa
        (lambda a, b: a if math.fabs(a) < math.fabs(b) else b),
        [diff, diff + math.pi * 2, diff - math.pi * 2]
    )


def normalisedTheta(theta):
    r = math.fmod(theta - math.pi, 2 * math.pi)
    if r > 0:
        return r - math.pi
    else:
        return r + math.pi


def doLineCollideCircle(circleX, circleY, radius, x1, y1, x2, y2):
    X1 = x1 - circleX
    X2 = x2 - circleX
    Y1 = y1 - circleY
    Y2 = y2 - circleY

    if X1 - X2 == 0:
        return math.fabs(X1) < radius
    else:
        m = (Y1 - Y2) / (X1 - X2)
        b = Y1 - m * X1
        return radius ** 2 * (1 + m ** 2) - b ** 2 > 0


def absToRr(fromPos, toPos):
    x_delta = toPos[0] - fromPos[0]
    y_delta = toPos[1] - fromPos[1]
    dist = math.hypot(x_delta, y_delta)
    heading = normalisedTheta(math.atan2(y_delta, x_delta) - fromPos[2])
    return dist, heading


def clamp(n, minn, maxn):
    return max(min(maxn, n), minn)
