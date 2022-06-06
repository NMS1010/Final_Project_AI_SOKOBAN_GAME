from re import X
import sys
from collections import deque
import time

from utilities import *

class BFSNode:
    def __init__(self, state, parent=None, action=None, path_cost=0):
        self.state = state
        self.parent = parent
        self.action = action
        self.path_cost = path_cost
        self.depth = 0
        if parent:
            self.depth = parent.depth + 1

    def expand(self, problem):
        return tuple([self.child_node(problem, action)
                for action in problem.actions(self.state)])

    def child_node(self, problem, action):
        next_state = problem.result(self.state, action)
        next_node = BFSNode(next_state, self, action, problem.path_cost(self.path_cost, self.state, action, next_state))
        return next_node

    def solution(self):
        node, path_back = self, []
        while node:
            path_back.append(node.state)
            node = node.parent
        return list(reversed(path_back))


def BFS(problem):
    start = time.time()
    
    node = BFSNode(problem.initial)
    if problem.goal_test(node.state):
        end = time.time()
        return node, (end-start)
    frontier = deque([node])
    explored = set()
    while frontier:
        node = frontier.popleft()
        explored.add(node.state)
        for child in node.expand(problem):
            if child.state not in explored and child not in frontier:
                if problem.goal_test(child.state):
                    end = time.time()
                    return child, (end-start)
                frontier.append(child)
        end = time.time()
        if end - start > 30:
            return False,(end-start)
    end = time.time()
    return None,(end-start)
      



    