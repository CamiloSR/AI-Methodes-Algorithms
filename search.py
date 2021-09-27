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
        python pacman.py -l tinyMaze -p SearchAgent -a fn=tinyMazeSearch
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return [s, s, w, s, w, w, s, w]


def depthFirstSearch(problem):
    """
    python pacman.py -l tinyMaze -p SearchAgent
    python pacman.py -l mediumMaze -p SearchAgent
    python pacman.py -l bigMaze -z .5 -p SearchAgent
    """
    from util import Stack

    # For DFS => LIFO Rules
    fringe = util.Stack()
    startNode = problem.getStartState()  # (n,n)

    if problem.isGoalState(startNode):  # Could happen
        print("Start Node " + startNode + " is the Goal State")
        return startNode

    fringe.push((startNode, [], []))

    while not fringe.isEmpty():  # SAME as fringe.isEmpty() == False
        nodeToVisit = fringe.pop()  # node, actions, visited = fringe.pop()
        node = nodeToVisit[0]
        actions = nodeToVisit[1]
        visited = nodeToVisit[2]

        if problem.isGoalState(node):
            print("The path is...")
            print(path)
            return path
        successors = problem.getSuccessors(node)

        for successor in successors:
            sNode = successor[0]
            sDirection = successor[1]
            sCost = successor[2]
            if not sNode in visited:
                fringe.push((sNode, actions + [sDirection], visited + [node]))
                path = actions + [sDirection]

    return []


def breadthFirstSearch(problem):
    """
    python pacman.py -l mediumMaze -p SearchAgent -a fn=bfs
    python pacman.py -l bigMaze -p SearchAgent -a fn=bfs -z .5
    """
    from util import Queue

    # For BFS the stak works with FIFO Rules so it is really a QUEUE
    fringe = util.Queue()
    startNode = problem.getStartState()  # (n,n)
    if problem.isGoalState(startNode):  # Could happen
        print("Start Node " + startNode + " is the Goal State")
        return startNode
    fringe.push((startNode, []))
    visited = []
    
    while not fringe.isEmpty():  # SAME as fringe.isEmpty() == False
        nodeToVisit = fringe.pop()  # node, actions, visited = fringe.pop()
        node = nodeToVisit[0]
        actions = nodeToVisit[1]
        successors = problem.getSuccessors(node)
        for successor in successors:
            sNode = successor[0]
            sDirection = successor[1]
            if sNode not in visited:
                if problem.isGoalState(sNode):
                    print("The path is...")
                    print(path)
                    return path
                fringe.push((sNode, actions + [sDirection]))
                path = actions + [sDirection]
                visited.append(sNode)

    return []

def uniformCostSearch(problem):
   """
    python pacman.py -l mediumMaze -p SearchAgent -a fn=ucs
    python pacman.py -l mediumDottedMaze -p StayEastSearchAgent
    python pacman.py -l mediumScaryMaze -p StayWestSearchAgent
    """
    from util import PriorityQueue

    fringe = util.PriorityQueue()
    visited = []
    startNode = problem.getStartState()  # (n,n)
    if problem.isGoalState(startNode):  # Could happen
        print("Start Node " + startNode + " is the Goal State")
        return startNode
    fringe.push((startNode, []), 0)
    visited.append(startNode)

    #input("Press Enter to continue...")

    while not fringe.isEmpty():  # SAME as fringe.isEmpty() == False
        nodeToVisit = fringe.pop() 
        node = nodeToVisit[0]
        actions = nodeToVisit[1]
        if problem.isGoalState(node):
            print("The path is...")
            print(actions)
            return actions
        if node not in visited:
            visited.append(node)

        successors = problem.getSuccessors(node)
        for successor in successors:
            sNode = successor[0]
            sDirection = successor[1]
            sCost = problem.getCostOfActions(actions + [sDirection])
            if sNode not in visited:
                fringe.update((sNode, actions + [sDirection]), sCost)

    return []


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0


def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    '''
        INSÉREZ VOTRE SOLUTION À LA QUESTION 4 ICI
    '''

    util.raiseNotDefined()


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
