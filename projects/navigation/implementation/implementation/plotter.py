from implementation.constants import *
from implementation.rrt import *
import matplotlib
matplotlib.use("TkAgg")
matplotlib.interactive(True)
import matplotlib.pyplot as plt

class Plotter:
    """
    A class that houses all the plotting functionality.

    Attributes
    ----------
    rrt: RRT
        the rapidly-exploring random tree that holds all the paths explored by
        the RRT algorithm.
    path: list
        the path RRT found from the initial state to the goal state.
    fig: matplotlib.figure.Figure
        the figure that'll be used to plot everything on
    ax: matplotlib.axes.SubplotBase
        the axes that'll be used to plot everything on

    Methods
    -------
    plot_rrt(wait: bool=False)
        Plots the full RRT. If wait=True, it shows the generation of the tree
        with some discretization
    plot_path()
        Plots the path from the initial state to the goal state found by RRT.
    plot_walls()
        Plots the walls.
    plot_robot()
        Plots the initial and final poses of the robot as well as its heading.
    plot_wheels()
        Plots the wheels of the robot for its initial and final poses.
    plot_all(wait: bool=False)
        Calls all the other plotting functions with wait reserved for plot_rrt()
    save_plot()
        Saves the plot in the figure.
    """
    rrt: RRT
    path: list
    fig: matplotlib.figure.Figure
    ax: matplotlib.axes.SubplotBase

    def __init__(self, rrt: RRT, path: list):
        self.rrt = rrt
        self.path = path
        self.fig = plt.figure(figsize=(10,10))
        self.ax = plt.subplot(111)
        self.ax.axis("equal")

    def plot_rrt(self, wait: bool=False):
        if wait:
            plt.pause(5)
            counter = 0
            for i in range(len(self.rrt.edges)):
                self.ax.plot([self.rrt.edges[i][0].p.x, self.rrt.edges[i][1].p.x], \
                    [self.rrt.edges[i][0].p.y,self.rrt.edges[i][1].p.y], "k-.", alpha=0.3)
                if counter % 500 == 0:
                    plt.pause(0.05)
                    counter = 0
                counter += 1
        else:
            for i in range(len(self.rrt.edges)):
                self.ax.plot([self.rrt.edges[i][0].p.x, self.rrt.edges[i][1].p.x], \
                    [self.rrt.edges[i][0].p.y,self.rrt.edges[i][1].p.y], "k-.", alpha=0.3)

    def plot_path(self):
        self.ax.plot([self.path[i].p.x for i in range(len(self.path))], \
            [self.path[i].p.y for i in range(len(self.path))], linewidth=3)

    def plot_walls(self):
        self.ax.plot([0, 2-epsilon], [0,0], "k", linewidth=4)
        self.ax.plot([2, 2], [0,2], "k", linewidth=4)
        self.ax.plot([0, 0], [0,2], "k", linewidth=4)
        self.ax.plot([0, 2], [2,2], "k", linewidth=4)
        self.ax.plot([2-epsilon, 2-epsilon], [0,1-epsilon/2], "k", linewidth=4)
        self.ax.plot([2-epsilon, 2-epsilon], [1+epsilon/2,3/2], "k", linewidth=4)
        self.ax.plot([1/2, 2-epsilon], [1-epsilon/2,1-epsilon/2], "k", linewidth=4)
        self.ax.plot([1/2, 2-epsilon], [1+epsilon/2,1+epsilon/2], "k", linewidth=4)

    def plot_robot(self):
        # Draw robot initial and final
        circle1 = plt.Circle((self.path[0].p.x, self.path[0].p.y), radius=0.1, color='g', fill=False, linewidth=3)
        circle2 = plt.Circle((self.path[-1].p.x, self.path[-1].p.y), radius=0.1, color='g', fill=False, linewidth=3)

        self.ax.add_patch(circle1)
        self.ax.add_patch(circle2)

        # Draw the directionality arrows
        self.ax.arrow(self.path[0].p.x, self.path[0].p.y, 0.1*sqrt(2)*cos(self.path[0].theta), \
                 0.1*sqrt(2)*sin(self.path[0].theta), color='g', linewidth=3, head_width=0.02)
        self.ax.arrow(self.path[-1].p.x, self.path[-1].p.y, 0.1*sqrt(2)*cos(self.path[-1].theta), \
                 0.1*sqrt(2)*sin(self.path[-1].theta), color='g', linewidth=3, head_width=0.02)


    def plot_wheels(self):
        # Wheel1
        w1 = [[-0.03, 0.03],[-0.1/sqrt(2), -0.1/sqrt(2)]]
        w2 = [[-0.03, 0.03],[0.1/sqrt(2), 0.1/sqrt(2)]]
        def rotatewheel(v, theta):
            tmpx0 = cos(theta)*v[0][0]-sin(theta)*v[1][0] 
            tmpy0 = sin(theta)*v[0][0]+cos(theta)*v[1][0]
            v[0][0], v[1][0] = tmpx0, tmpy0
            tmpx1 = cos(theta)*v[0][1]-sin(theta)*v[1][1] 
            tmpy1 = sin(theta)*v[0][1]+cos(theta)*v[1][1]
            v[0][1], v[1][1] = tmpx1, tmpy1
            return v

        def translatewheel(v, p):
            v[0][0] += p[0]
            v[1][0] += p[1]
            v[0][1] += p[0]
            v[1][1] += p[1]
            return v

        w1 = rotatewheel(w1, self.path[0].theta)
        w2 = rotatewheel(w2, self.path[0].theta)
        w1 = translatewheel(w1, [self.path[0].p.x, self.path[0].p.y])
        w2 = translatewheel(w2, [self.path[0].p.x, self.path[0].p.y])

        self.ax.plot(w1[0], w1[1], linewidth=3, color='g')
        self.ax.plot(w2[0], w2[1], linewidth=3, color='g')

        # Wheel2
        w1 = [[-0.03, 0.03],[-0.1/sqrt(2), -0.1/sqrt(2)]]
        w2 = [[-0.03, 0.03],[0.1/sqrt(2), 0.1/sqrt(2)]]

        w1 = rotatewheel(w1, self.path[-1].theta)
        w2 = rotatewheel(w2, self.path[-1].theta)
        w1 = translatewheel(w1, [self.path[-1].p.x, self.path[-1].p.y])
        w2 = translatewheel(w2, [self.path[-1].p.x, self.path[-1].p.y])

        self.ax.plot(w1[0], w1[1], linewidth=3, color='g')
        self.ax.plot(w2[0], w2[1], linewidth=3, color='g')

    def plot_all(self, wait: bool=False):
        self.plot_walls();
        self.plot_wheels();
        self.plot_robot();
        self.plot_rrt(wait);
        self.plot_path();
        self.fig.show()

    def save_plot(self):
        self.fig.savefig("rrt.svg", format="svg", bbox_inches="tight")
