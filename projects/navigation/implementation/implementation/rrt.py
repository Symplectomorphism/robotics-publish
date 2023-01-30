import copy
from implementation.kinematics import *

class RRT:
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

    def new_state(self, snear: State, deltat: float=0.01):
        self.r.set_state(copy.deepcopy(snear))
        actions = [Action(-1,-1), Action(-1, 0), Action(-1,1),
                   Action(0,-1), Action(0, 0), Action(0,1),
                   Action(1,-1), Action(1, 0), Action(1,1)]

#         actions = [Action(-1, 0),
#                    Action(0,-1), Action(0, 0), Action(0,1),
#                    Action(1, 0)]

        bestcost = snear.dist(self.prand)
        bests = copy.deepcopy(snear)
        besta = Action(0,0)

        for a in actions:
            try: 
                s = self.r.simulate(deltat, a)
            except CollisionException:
                self.r.set_state(copy.deepcopy(snear))
                continue
            cost = s.dist(self.prand)
            if cost < bestcost:
                bestcost = cost
                bests = s
                besta = a
            self.r.set_state(copy.deepcopy(snear))

        if bests != snear:
            self.vertices.append(bests)
            self.edges.append([snear, bests, besta])
            if dist(bests.p, self.r.pgoal) < 0.1:
                return "Reached"
            else:
                return "Advanced"
        else:
            return "Trapped"

    def nearest_neighbor(self, x: Point):
        bestdist = self.vertices[0].dist(x)
        snear = self.vertices[0]
        for v in self.vertices:
            cur_dist = v.dist(x)
            if cur_dist < bestdist:
                bestdist = cur_dist
                snear = v
        return copy.deepcopy(snear)

    def extend(self):
        self.prand = random_state().p
        snear = self.nearest_neighbor(self.prand)
        return self.new_state(snear, deltat=0.05)

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

