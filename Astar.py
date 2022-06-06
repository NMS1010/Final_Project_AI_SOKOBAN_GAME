from queue import PriorityQueue
import time

from utilities import *

# Tao mot class chua thong tin ve mot vi tri trong maze
class AstarNode:
    def __init__(self, state, goal, parent = None):
        self.state = state
        self.parent = parent
        self.goal = goal
        self.g = 0 # path cost
        self.h = 0 # heuristic: dung straight-line distance
        self.f = 0 # f(n) = g(n) + h(n)
    def cal_heuristic(self):
        box_poses = get_box_position(self.state)
        for p in box_poses:
            self.h += get_goal_nearest_box_dis(p, self.goal)
    def expand(self, problem):
        return tuple([self.child_node(problem, action)
                for action in problem.actions(self.state)])

    def child_node(self, problem, action):
        next_state = problem.result(self.state, action)
        next_node = AstarNode(next_state, self.goal, self)
        return next_node
    def cal_f(self):
        '''Tinh toan he so F dua vao viec tinh he so g va h'''
        self.g = self.parent.g + 1
        self.cal_heuristic()
        self.f = self.g + self.h

    def solution(self):
        '''Truy vet duong di'''
        path = []
        traversal = self
        while traversal:
            path.append(traversal.state)
            traversal = traversal.parent
        return list(reversed(path))
    def __gt__(self, node):
        return self.f > node.f
    def __lt__(self, node):
        return self.f < node.f

def astar(problem):
    start = time.time()
    startNode = AstarNode(problem.initial, problem.goal)
    frontier = PriorityQueue()
    frontier.put(startNode)
    explored = set()
    while frontier:
        currNode = frontier.get()
        if problem.goal_test(currNode.state):
            end = time.time()
            return currNode, (end - start)
        explored.add(currNode.state)
        for child in currNode.expand(problem):
            child.cal_f()
            node = get_value_contain_in_PrioQueue(child, frontier)
            if child.state not in explored and node == False:
                frontier.put(child)
            elif node != False:
                if child.f < node.f:
                    frontier.queue.remove(node)
                    frontier.put(child)
        end = time.time()
        if end - start > 30:
            return False,(end-start)
    end = time.time()
    return None, (end - start)
