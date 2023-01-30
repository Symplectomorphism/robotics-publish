import copy
from implementation.kinematics import *
from implementation.occupancy import Point
from implementation.rrt import *
from math import pi

t = 0.1
p = Point(5/4, 1/4)
theta = 60*pi/180
a = Action(0, 0)
r = Robot(copy.deepcopy(p), theta,a);
ptime = 0.0

# t = r.move_straight(Point(1/4,1/4), t, pausetime=ptime)
# print(f"Time: {t}")
# t = r.move_straight(Point(1/4,7/4), t, pausetime=ptime)
# print(f"Time: {t}")
# t = r.move_straight(Point(2-0.15,7/4), t, pausetime=ptime)
# print(f"Time: {t}")
# t = r.move_straight(Point(2-0.15,0), t, pausetime=ptime)
# print(f"Time: {t}")

""" Try below for collision error """
# t = r.move_straight(Point(1.8,1/4), t, pausetime=0.001)

rrt = RRT(p, theta)
i, counter = 0, 0
while rrt.extend() != "Reached":
    if counter % 100 == 0:
        print(f"Iteration: {i}, NN: {rrt.nearest_neighbor(r.pgoal)}")
        print(f"Last entry: {rrt.vertices[-1]}")
        counter = 0
    counter += 1
    i += 1

print(len(rrt.vertices))
print(len(rrt.edges))
