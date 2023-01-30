from implementation.constants import epsilon, delta, r

class Point:
    x: float
    y: float

    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __str__(self):
        return f"x: {self.x:.3f}, y: {self.y:.3f}"

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y

def isoccupied(point: Point) -> bool:
    flag = False
    if ((0 <= point.x <= delta+r) or
        (2-delta-r <= point.x <= 2) or 
        (0 <= point.x <= 2-epsilon-r and 0 <= point.y <= delta+r) or
        (2-delta-r <= point.y <= 2) or
        (2-epsilon-r <= point.x <= 2-epsilon+delta+r and 0 <= point.y <= 1-epsilon/2+r) or
        (2-epsilon-r <= point.x <= 2-epsilon+delta+r and 1+epsilon/2-r <= point.y <= 3/2+r) or
        (1/2-r <= point.x <= 2-epsilon+r and 1-epsilon/2-r <= point.y <= 1-epsilon/2+delta+r) or
        (1/2-r <= point.x <= 2-epsilon+r and 1+epsilon/2-r <= point.y <= 1+epsilon/2+delta+r)
        ):
        flag = True
    return flag 
