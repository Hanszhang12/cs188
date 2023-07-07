# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

import util

class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]

class Node:
    def __init__(self, direction, moves, coordinates):
        self.actions = moves
        self.direction = direction
        self.actions = self.actions + [direction]
        self.state = coordinates 


    def getState(self):
        return self.state

    def getDirections(self):
        return self.actions

    def updateDirection(self, newActions, newDirection):
        self.actions = newActions + [newDirection]


def depthFirstSearch(problem):
    from game import Directions
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:
    """
    """
    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors(problem.getStartState()))"""



    """Nodes store the directions needed to get to the state"""
    """Visited array keeps track of all the coordinates of visited nodes"""

    stack = util.Stack()
    exploredNodes = []
    visitedNodes = []

    currentState = Node(Directions.STOP, [Directions.STOP], problem.getStartState())

    while not problem.isGoalState(currentState.getState()):
        check = False
        for visited in exploredNodes:
            if currentState.getState() == visited:
                check = True

        if not check:
            exploredNodes.append(currentState.getState())

            previousDirections = currentState.getDirections()

            for s in problem.getSuccessors(currentState.getState()):
                check2 = False
                for visited in visitedNodes:
                    if s[0] == visited:
                        check2 = True
                if not check2:
                    node = Node(s[1], previousDirections, s[0])
                    stack.push(node)
                    visitedNodes.append(s[0])
                else:
                    for p in stack.list:
                        if s[0] == p.getState():
                            p.updateDirection(previousDirections, s[1])

        currentState = stack.pop()

    return currentState.getDirections()[2:]


def visitedCheck(tup, visited):
    for i in visited:
        if tup == i:
            return False
    return True

  
def breadthFirstSearch(problem):
    from game import Directions
    """Search the shallowest nodes in the search tree first."""
    queue = util.Queue()
    exploredNodes = []
    visitedNodes = []
    currentState = Node(Directions.STOP, [Directions.STOP], problem.getStartState())
    while not problem.isGoalState(currentState.getState()):
        check = False
        for visited in exploredNodes:
            if currentState.getState() == visited:
                check = True
        if not check:
            exploredNodes.append(currentState.getState())

            previousDirections = currentState.getDirections()

            for s in problem.getSuccessors(currentState.getState()):
                check2 = False
                for visited in visitedNodes:
                    if s[0] == visited:
                        check2 = True
                if not check2:
                    node = Node(s[1], previousDirections, s[0])
                    queue.push(node)
                    visitedNodes.append(s[0])
                else:
                    for p in queue.list:
                        if s[0] == p.getState() and (len(previousDirections) + 1 < len(p.getDirections())):
                            p.updateDirection(previousDirections, s[1])

        currentState = queue.pop()
    return currentState.getDirections()[2:]

def uniformCostSearch(problem):
    from game import Directions
    """Search the node of least total cost first."""
    class Node:
        def __init__(self, direction, moves, coordinates, priority):
            self.actions = moves
            self.direction = direction
            self.actions = self.actions + [direction]
            self.state = coordinates
            self.priority = priority
            
        def getState(self):
            return self.state

        def getPriority(self):
            return self.priority

        def getDirections(self):
            return self.actions

        def updateDirection(self, newActions, newDirection):
            self.actions = newActions + [newDirection]

    pq = util.PriorityQueue()
    visitedNodes = []
    exploredNodes = []
    currentState = Node(Directions.STOP, [Directions.STOP], problem.getStartState(), 0)
    while not problem.isGoalState(currentState.getState()):
        check = False
        for visited in exploredNodes:
            if currentState.getState() == visited:
                check = True

        if not check:
            exploredNodes.append(currentState.getState())

            previousDirections = currentState.getDirections()

            for s in problem.getSuccessors(currentState.getState()):
                node = Node(s[1], previousDirections, s[0], s[2] + currentState.getPriority())
                pq.push(node, node.getPriority())

        currentState = pq.pop()
    return currentState.getDirections()[2:]


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""

    from game import Directions
    """Search the node of least total cost first."""
    class Node:
        def __init__(self, direction, moves, coordinates, priority):
            self.actions = moves
            self.direction = direction
            self.actions = self.actions + [direction]
            self.state = coordinates
            self.priority = priority
            
        def getState(self):
            return self.state

        def getPriority(self):
            return self.priority

        def getDirections(self):
            return self.actions

        def updateDirection(self, newActions, newDirection):
            self.actions = newActions + [newDirection]
    
    pq = util.PriorityQueue()
    visitedNodes = []
    exploredNodes = []
    currentState = Node(Directions.STOP, [Directions.STOP], problem.getStartState(), 0)
    while not problem.isGoalState(currentState.getState()):
        check = False
        for visited in exploredNodes:
            if currentState.getState() == visited:
                check = True

        if not check:
            exploredNodes.append(currentState.getState())

            previousDirections = currentState.getDirections()
            for s in problem.getSuccessors(currentState.getState()):
                node = Node(s[1], previousDirections, s[0], currentState.getPriority() + s[2])
                pq.push(node, node.getPriority() + heuristic(s[0], problem))
        currentState = pq.pop()
        
    return currentState.getDirections()[2:]




# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
