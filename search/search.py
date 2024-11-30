import util
from util import Queue, PriorityQueue

class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).
    """
    def getStartState(self):
        util.raiseNotDefined()

    def isGoalState(self, state):
        util.raiseNotDefined()

    def getSuccessors(self, state):
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze. For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]


def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.

    This implementation is based on DFS with graph search to avoid cycles.
    """
    startState = problem.getStartState()
    if problem.isGoalState(startState):
        return []

    frontier = util.Stack()
    frontier.push((startState, []))  # Push (state, path) onto the frontier
    explored = set()  # Set of explored states

    while not frontier.isEmpty():
        state, path = frontier.pop()  # Pop the last state and its path
        if state in explored:
            continue
        explored.add(state)

        if problem.isGoalState(state):
            return path

        for successor, action, cost in problem.getSuccessors(state):
            if successor not in explored:
                frontier.push((successor, path + [action]))

    return []  # Return an empty list if no solution is found


def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    startState = problem.getStartState()
    if problem.isGoalState(startState):
        return []

    frontier = Queue()
    frontier.push((startState, []))  # Push (state, path) onto the frontier
    explored = set()  # Set of explored states

    while not frontier.isEmpty():
        state, path = frontier.pop()  # Pop the first state and its path
        if state in explored:
            continue
        explored.add(state)

        if problem.isGoalState(state):
            return path

        for successor, action, cost in problem.getSuccessors(state):
            if successor not in explored:
                frontier.push((successor, path + [action]))

    return []  # Return an empty list if no solution is found


def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    startState = problem.getStartState()
    if problem.isGoalState(startState):
        return []

    frontier = PriorityQueue()
    frontier.push((startState, []), 0)  # Push (state, path) with priority of 0
    explored = set()  # Set of explored states

    while not frontier.isEmpty():
        state, path = frontier.pop()  # Pop the state and its path
        if state in explored:
            continue
        explored.add(state)

        if problem.isGoalState(state):
            return path

        for successor, action, cost in problem.getSuccessors(state):
            if successor not in explored:
                totalCost = problem.getCostOfActions(path + [action])
                frontier.push((successor, path + [action]), totalCost)

    return []  # Return an empty list if no solution is found


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0


def aStarSearch(problem, heuristic=nullHeuristic):
 
    

    return []  # Return an empty list if no solution is found


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
