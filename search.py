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
import searchAgents

class Node:
    "A node is a state and the path leading to it (described as actions)."
    def __init__(self, state, path = []):
        self.state = state
        self.path = path.copy()

    def __eq__(self, other):
        return (isinstance(other, self.__class__)
                and self.state == other.state)


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
    return [s, s, w, s, w, w, s, w]


def depthFirstSearch(problem: SearchProblem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors(problem.getStartState()))
    """
    closed = []  # list of closed states
    frontier = util.Stack()
    frontier.push(Node(problem.getStartState()))  # stack of nodes

    while frontier:
        node = frontier.pop()  # node is <state, path>

        if problem.isGoalState(node.state):
            return node.path
        closed.append(node.state)
        successors = problem.getSuccessors(node.state)  # <successorState, action, cost>
        for succ in successors:
            path = node.path.copy()
            path.append(succ[1])
            newNode = Node(succ[0], path)
            if newNode.state not in closed:
                frontier.push(newNode)
    return None


def breadthFirstSearch(problem: SearchProblem):
    """Search the shallowest nodes in the search tree first."""
    closed = []  # list of closed states
    frontier = util.Queue()
    frontier.push(Node(problem.getStartState()))  # queue of nodes

    while frontier:
        node = frontier.pop()  # node is <state, path>

        if problem.isGoalState(node.state):
            return node.path
        closed.append(node.state)
        successors = problem.getSuccessors(node.state)  # <successorState, action, cost>
        for succ in successors:
            path = node.path.copy()
            path.append(succ[1])
            newNode = Node(succ[0], path)
            if newNode.state not in closed:
                frontier.push(newNode)
    return None


def uniformCostSearch(problem: SearchProblem):
    """Search the node of least total cost first."""
    closed = []  # list of closed states
    frontier = util.PriorityQueueWithFunction(lambda some_node: problem.getCostOfActions(some_node.path))
    frontier.push(Node(problem.getStartState()))

    while frontier:
        node = frontier.pop()  # node is <state, path>

        if problem.isGoalState(node.state):
            return node.path
        closed.append(node.state)
        successors = problem.getSuccessors(node.state)  # <successorState, action, cost>
        for succ in successors:
            new_path = node.path.copy()
            new_path.append(succ[1])
            newNode = Node(succ[0], new_path)
            if newNode not in closed:
                #  frontier.update(newNode, problem.getCostOfActions(newNode.path))
                frontier.push(newNode)
    return None


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0


def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""

    closed = []  # list of closed states
    frontier = util.PriorityQueueWithFunction(lambda state: problem.getCostOfActions(state[1]) + heuristic(state[0],problem))
    frontier.push([problem.getStartState(), []])  # stack of nodes, node = <state, path>

    while frontier:
        node = frontier.pop()  # node is <state, path>

        if problem.isGoalState(node[0]):
            return node[1]
        closed.append(node[0])
        successors = problem.getSuccessors(node[0])  # <successorState, action, cost>
        for succ in successors:
            path = node[1].copy()
            path.append(succ[1])
            newNode = [succ[0], path]
            if newNode[0] not in closed:
                frontier.push(newNode)
    return None

# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
