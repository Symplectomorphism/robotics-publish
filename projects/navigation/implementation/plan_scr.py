from implementation.rrt import *
from implementation.plotter import Plotter
from math import pi
import pickle

p = Point(5/4, 1/4)
theta = 60*pi/180

""" Uncomment if want to regenerate RRT """
# rrt = RRT(p, theta)
# i, counter = 0, 0
# while rrt.extend(sense=0) != "Reached":
#     if counter % 100 == 0:
#         print(f"Iteration: {i}, NN: {rrt.nearest_neighbor(rrt.r.pgoal)}")
#         print(f"Last entry: {rrt.vertices[-1]}")
#         counter = 0
#     counter += 1
#     i += 1

""" To save a generated RRT """
# file = open("rrt_start_left.obj", "wb")
# pickle.dump(rrt, file)
""" To load an already saved RRT """
file = open("rrt_start_bottom_right.obj", "rb")
rrt = pickle.load(file)
path = rrt.findpath()

""" Plot the RRT """
plotter = Plotter(rrt, path)
plotter.plot_all(wait=True)
