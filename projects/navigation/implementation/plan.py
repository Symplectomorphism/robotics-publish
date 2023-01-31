from implementation.rrt import *
from implementation.plotter import Plotter
from math import pi

def main():
    p = Point(5/4, 1/4)
    theta = 60*pi/180

    rrt = RRT(p, theta)
    i, counter = 0, 0
    while rrt.extend(sense=1) != "Reached":
        if counter % 100 == 0:
            print(f"Iteration: {i}, NN: {rrt.nearest_neighbor(rrt.r.pgoal)}")
            print(f"Last entry: {rrt.vertices[-1]}")
            counter = 0
        counter += 1
        i += 1
    path = rrt.findpath()

    """ Plot the RRT """
    plotter = Plotter(rrt, path)
    plotter.plot_all(wait=False)
    plotter.save_plot()
