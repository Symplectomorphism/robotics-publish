import copy
import time
import random
random.seed(1)

from implementation.constants import epsilon
from implementation.occupancy import isoccupied, Point
from math import cos, sin, pi, atan2, floor, sqrt
import numpy as np

class CollisionException(Exception):
    """Raised when there is a collision"""
    pass

class State:
    p: Point
    theta: float

    def __init__(self, p: Point, theta: float):
        self.p = p
        self.theta = theta

    def dist(self, q: Point):
        vx, vy = q.x - self.p.x, q.y - self.p.y
        mag = sqrt(vx**2 + vy**2)
        vx, vy = vx/mag, vy/mag
        dir_cost = 1 - (vx*cos(self.theta) + vy*sin(self.theta))
        return max(abs(self.p.x-q.x), abs(self.p.y-q.y)) + dir_cost

    def __str__(self):
        return f"x: {self.p.x:.3f}, y: {self.p.y:.3f}, th: {self.theta*180/pi:.3f}"

    def __eq__(self, other):
        return self.p.x == other.p.x and self.p.y == other.p.y and self.theta == other.theta

class Action:
    v: float
    omega: float

    def __init__(self, v: float, omega: float):
        self.v = v
        self.omega = omega

    def __str__(self):
        return f"v: {self.v}, omega: {self.omega}"

class Robot:
    s: State
    a: Action
    dt: float
    pgoal: Point

    def __init__(self, p: Point, theta: float=0, a: Action=Action(0,0)):
        self.s = State(p, theta)
        self.a = a
        self.dt = 0.001
        self.pgoal = Point(2-epsilon/2, 0)

    def __str__(self):
        return f"Robot state: {self.s.p.x:.2f}, {self.s.p.y:.2f}, {self.s.theta*180/pi:.2f}\nAction: {self.a.v},{self.a.omega}"

    def set_state(self, s: State):
        self.s = s

    def step(self):
        """ Non-holonomic robot """
        xnext = self.s.p.x + self.a.v * cos(self.s.theta) * self.dt
        ynext = self.s.p.y + self.a.v * sin(self.s.theta) * self.dt

        """ Holonomic robot """
        # xnext = self.s.p.x + self.a.v * self.dt
        # ynext = self.s.p.y + self.a.omega * self.dt

        if not isoccupied(Point(xnext,ynext)):
            self.s.p.x = xnext
            self.s.p.y = ynext
            self.s.theta += self.a.omega * self.dt
        else:
            raise CollisionException
    
    def simulate(self, interval: float=0.001, a: Action=Action(0,0)) -> State:
        num_steps = floor(int(interval/self.dt))
        self.a = copy.deepcopy(a)
        for _ in range(num_steps):
            self.step()
        return copy.deepcopy(self.s)

    def move_straight(self, target: Point, t: float=0, pausetime: float=0):
        theta = atan2(target.y-self.s.p.y, target.x-self.s.p.x)
        e = self.s.theta-theta
        counter = 0
        while abs(e) > 1e-3:
            self.a.v = 0
            self.a.omega = -np.sign(e)
            self.step()
            e = self.s.theta - theta
            if counter % 100 == 0:
                print(self)
                counter = 0
            counter += 1
            t += self.dt
            time.sleep(pausetime)
        e = dist(target, self.s.p)
        counter = 0;
        while abs(e) > 1e-3:
            self.a.v = 1
            self.a.omega = 0
            try:
                self.step()
            except CollisionException:
                print("Collision detected, cannot proceed!")
                break
            e = dist(target, self.s.p)
            if counter % 100 == 0:
                print(self)
                counter = 0
            counter += 1
            t += self.dt
            time.sleep(pausetime)
        return t

def dist(p: Point, q: Point, sense: int=1):
    if sense == 1:
        return max(abs(q.x-p.x), abs(q.y-p.y))
    else:
        return np.linalg.norm([q.x-p.x, q.y-p.y])

def random_state() -> State:
    p = Point(*(2*random.random() for _ in [1,2]))
    while isoccupied(p):
        p = Point(*(2*random.random() for _ in [1,2]))
    return State(p, -pi + 2*pi*random.random())
