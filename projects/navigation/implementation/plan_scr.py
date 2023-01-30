from implementation.rrt import *
from math import pi
import matplotlib
matplotlib.use("TkAgg")
matplotlib.interactive(True)
import matplotlib.pyplot as plt
import pickle

p = Point(1/4, 4/4)
theta = 60*pi/180

""" Uncomment if want to regenerate RRT """
# rrt = RRT(p, theta)
# i, counter = 0, 0
# while rrt.extend() != "Reached":
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
fig = plt.figure(figsize=(10,6))
ax = plt.subplot(1,1,1)

# for i in range(len(rrt.vertices)):
#     ax.plot(rrt.vertices[i].p.x, rrt.vertices[i].p.y,".")
for i in range(len(rrt.edges)):
    ax.plot([rrt.edges[i][0].p.x, rrt.edges[i][1].p.x], \
            [rrt.edges[i][0].p.y,rrt.edges[i][1].p.y], "k-.", alpha=0.4)
ax.plot([path[i].p.x for i in range(len(path))], \
        [path[i].p.y for i in range(len(path))], linewidth=3)
fig.show()
fig.savefig("rrt.svg", format="svg", bbox_inches="tight")
