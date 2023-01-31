import copy
from implementation.kinematics import *

class RRT:
    """
    A class that implements the RRT algorithm.

    Attributes
    ----------
    r: Robot
        holds an instance of the wheeled mobile robot.
    vertices: list
        holds the vertices of RRT; this is a list of robot states.
    edges: list
        holds the edges of the RRT; this is a list of lists. It contains the
        start state and the end edge as well as the control input used to arrive
        at the end state.
    prand: Point
        Holds the random point used to generate the next edge and vertex in RRT.

    Methods
    -------
    new_state(snear: State, deltat: float, sense: int)
        Given the next random point and the nearest element in the current RRT,
        finds a new state that is closer to the random point. If it can find a
        closer state, returns "Advanced," if the new state is close enough to
        the goal state, returns "Reached,"  if no progress can be made towards
        the random point, returns "Trapped." deltat determines the simulation
        interval and sense determines which distance function to use.
    nearest_neighbor(x: Point, sense: int)
        Given a point x, finds the nearest state in the current RRT to that
        point. The nearness is measured by the distance, which is determined by
        the sense argument.
    extend(sense: int)
        Calles the nearest_neighbor and new_state functions while passing the
        sense argument forward.
    findvertex(s: State) -> int
        Given a state, finds it in the RRT vertex list. Returns the index in
        which it is found. If not found, returns -1.
    findedge(s: State) -> int
        Given a state, finds the edge that is incident to the vertex that
        represents that state in RRT. Returns the index of this edge if the
        vertex that corresponds to the state s is found. If not, returns -1.
    findpath() -> list
        Finds the path in the RRT that starts from the initial robot state to
        the goal state.
    """

    r: Robot
    vertices: list  # List of vertices -- which are states
    edges: list     # List of edges -- which are pairs of states
    path: list      # List of edges that form the path from start to finish
    prand: Point

    def __init__(self, p: Point, theta: float=0.0):
        self.r = Robot(p, theta)
        self.vertices = list()
        self.vertices.append(self.r.s)
        self.edges = list()
        self.prand = self.r.pgoal

    def new_state(self, snear: State, deltat: float=0.01, sense: int=0):
        self.r.set_state(copy.deepcopy(snear))
        actions = [Action(-1,-1), Action(-1, 0), Action(-1,1),
                   Action(0,-1), Action(0, 0), Action(0,1),
                   Action(1,-1), Action(1, 0), Action(1,1)]

#         actions = [Action(-1, 0),
#                    Action(0,-1), Action(0, 0), Action(0,1),
#                    Action(1, 0)]

        bestcost = snear.dist(self.prand, sense=sense)
        bests = copy.deepcopy(snear)
        besta = Action(0,0)

        for a in actions:
            try: 
                s = self.r.simulate(deltat, a)
            except CollisionException:
                self.r.set_state(copy.deepcopy(snear))
                continue
            cost = s.dist(self.prand, sense=sense)
            if cost < bestcost:
                bestcost = cost
                bests = s
                besta = a
            self.r.set_state(copy.deepcopy(snear))

        if bests != snear:
            self.vertices.append(bests)
            self.edges.append([snear, bests, besta])
            if dist(bests.p, self.r.pgoal, sense=sense) < 0.1:
                return "Reached"
            else:
                return "Advanced"
        else:
            return "Trapped"

    def nearest_neighbor(self, x: Point, sense: int=0):
        bestdist = self.vertices[0].dist(x, sense=sense)
        snear = self.vertices[0]
        for v in self.vertices:
            cur_dist = v.dist(x, sense=1)
            if cur_dist < bestdist:
                bestdist = cur_dist
                snear = v
        return copy.deepcopy(snear)

    def extend(self, sense: int=0):
        self.prand = random_state().p
        snear = self.nearest_neighbor(self.prand, sense=sense)
        return self.new_state(snear, deltat=0.05, sense=sense)

    def findvertex(self, s: State) -> int:
        for i in range(len(self.vertices)):
            if self.vertices[i] == s:
                return i
        return -1

    def findedge(self, s:State) -> int:
        for i in range(len(self.edges)):
            if self.edges[i][1] == s:
                return i
        return -1

    def findpath(self) -> list:
        s = list()
        ind = self.findedge(self.nearest_neighbor(self.r.pgoal))
        s.insert(0, self.edges[ind][1])
        while self.edges[ind][0] != self.vertices[0]:
            ind = self.findedge(self.edges[ind][0])
            s.insert(0, self.edges[ind][1])
        return s

