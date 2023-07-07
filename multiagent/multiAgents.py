# multiAgents.py
# --------------
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


from util import manhattanDistance
from game import Directions
import random, util

from game import Agent

class ReflexAgent(Agent):
    """
    A reflex agent chooses an action at each choice point by examining
    its alternatives via a state evaluation function.

    The code below is provided as a guide.  You are welcome to change
    it in any way you see fit, so long as you don't touch our method
    headers.
    """


    def getAction(self, gameState):
        """
        You do not need to change this method, but you're welcome to.

        getAction chooses among the best options according to the evaluation function.

        Just like in the previous project, getAction takes a GameState and returns
        some Directions.X for some X in the set {NORTH, SOUTH, WEST, EAST, STOP}
        """
        # Collect legal moves and successor states
        legalMoves = gameState.getLegalActions()

        # Choose one of the best actions
        scores = [self.evaluationFunction(gameState, action) for action in legalMoves]
        bestScore = max(scores)
        bestIndices = [index for index in range(len(scores)) if scores[index] == bestScore]
        chosenIndex = random.choice(bestIndices) # Pick randomly among the best

        "Add more of your code here if you want to"

        return legalMoves[chosenIndex]

    def evaluationFunction(self, currentGameState, action):
        """
        Design a better evaluation function here.

        The evaluation function takes in the current and proposed successor
        GameStates (pacman.py) and returns a number, where higher numbers are better.

        The code below extracts some useful information from the state, like the
        remaining food (newFood) and Pacman position after moving (newPos).
        newScaredTimes holds the number of moves that each ghost will remain
        scared because of Pacman having eaten a power pellet.

        Print out these variables to see what you're getting, then combine them
        to create a masterful evaluation function.
        """
        # Useful information you can extract from a GameState (pacman.py)
        successorGameState = currentGameState.generatePacmanSuccessor(action)
        newPos = successorGameState.getPacmanPosition()
        newFood = successorGameState.getFood()
        newGhostStates = successorGameState.getGhostStates()
        newScaredTimes = [ghostState.scaredTimer for ghostState in newGhostStates]

        score = successorGameState.getScore()

        if successorGameState.isLose():
            return -float("inf")
        elif successorGameState.isWin():
            return float("inf")

        foodDist = []
        for food in list(newFood.asList()):
            foodDist.append(util.manhattanDistance(food, newPos))

        for ghost in newGhostStates:
            ghostDist = util.manhattanDistance(ghost.getPosition(), newPos)
            if ghostDist < 1 and ghost.scaredTimer ==0:
                return 0
            elif (ghost.scaredTimer == 1):
                return float("inf")


        return score + (1 / min(foodDist))

    

def scoreEvaluationFunction(currentGameState):
    """
    This default evaluation function just returns the score of the state.
    The score is the same one displayed in the Pacman GUI.

    This evaluation function is meant for use with adversarial search agents
    (not reflex agents).
    """
    return currentGameState.getScore()

class MultiAgentSearchAgent(Agent):
    """
    This class provides some common elements to all of your
    multi-agent searchers.  Any methods defined here will be available
    to the MinimaxPacmanAgent, AlphaBetaPacmanAgent & ExpectimaxPacmanAgent.

    You *do not* need to make any changes here, but you can if you want to
    add functionality to all your adversarial search agents.  Please do not
    remove anything, however.

    Note: this is an abstract class: one that should not be instantiated.  It's
    only partially specified, and designed to be extended.  Agent (game.py)
    is another abstract class.
    """

    def __init__(self, evalFn = 'scoreEvaluationFunction', depth = '2'):
        self.index = 0 # Pacman is always agent index 0
        self.evaluationFunction = util.lookup(evalFn, globals())
        self.depth = int(depth)

class MinimaxAgent(MultiAgentSearchAgent):
    """
    Your minimax agent (question 2)
    """

    def getAction(self, gameState):
        """
        Returns the minimax action from the current gameState using self.depth
        and self.evaluationFunction.

        Here are some method calls that might be useful when implementing minimax.

        gameState.getLegalActions(agentIndex):
        Returns a list of legal actions for an agent
        agentIndex=0 means Pacman, ghosts are >= 1

        gameState.generateSuccessor(agentIndex, action):
        Returns the successor game state after an agent takes an action

        gameState.getNumAgents():
        Returns the total number of agents in the game

        gameState.isWin():
        Returns whether or not the game state is a winning state

        gameState.isLose():
        Returns whether or not the game state is a losing state
        """

        "*** YOUR CODE HERE ***"
        """
        pseudocode 
        def value(state):
            if the state is terminal state: return the state's utility
            if the next agent is MAX: return max-value(state)
            if the next agent is MIN: return min-value(state)

        def max-value(state):
            initialize v = - inf
            for each sucessor of state:
                v = max(v, value(successor))
            return v

        def min-value(state):
            initialize v = inf
            for each successor of state:
                v = min(v, value(successor))
            return v
        """
        index = 0
        numAgents = gameState.getNumAgents()
        legalActions = gameState.getLegalActions(index)
        val = -float("inf")
        currentAction = None
        for action in list(legalActions):
            successor = gameState.generateSuccessor(index, action)
            temp = self.value(successor, 1, 0)
            if max(temp, val) == temp:
                val = temp
                currentAction = action
        return currentAction

    def value(self, state, index, depth):
        if state.isWin() or state.isLose():
            return self.evaluationFunction(state)
        if depth == self.depth:
            return self.evaluationFunction(state)
        if index == 0:
            return self.max_value(state, index, depth)
        else: 
            return self.min_value(state, index, depth)

    def max_value(self, state, index, depth):
        v = -(float("inf"))
        legalActions = state.getLegalActions(index)
        for action in list(legalActions):
            successor = state.generateSuccessor(index, action)
            temp = self.value(successor, 1, depth)
            if v < temp:
                v = temp
        return v

    def min_value(self, state, index, depth):
        v = float("inf")
        legalActions = state.getLegalActions(index)
        for action in list(legalActions):
            successor = state.generateSuccessor(index, action)
            ghosts = state.getNumAgents() - 1
            if index == ghosts:
                temp = self.value(successor, 0, depth + 1)
            else: 
                temp = self.value(successor, index + 1, depth)
            if temp < v:
                v = temp
        return v 
        



class AlphaBetaAgent(MultiAgentSearchAgent):
    """
    Your minimax agent with alpha-beta pruning (question 3)
    """

    def getAction(self, gameState):
        """
        Returns the minimax action using self.depth and self.evaluationFunction
        """
        "*** YOUR CODE HERE ***"

        """pseudocode
        a: MAX's best option on path to root
        b: MIN's best option on path to root 

        def max-value(state, a, b):
            initialize v = -inf
            for each successor of state:
                v = max(v, value(successor, a, b))
                if v > b return v
                a = max(a, v)
            return v 

        def min-value(state, a, b):
            initialize v = inf
            for each successor of state:
                v = min(v, value(successor, a, b))
                if v < a return v
                b = min(b, v)
            return v 
        """
        index = 0
        a = -float("inf")
        b = float("inf")
        numAgents = gameState.getNumAgents()
        legalActions = gameState.getLegalActions(index)
        val = -float("inf")
        currentAction = None
        for action in list(legalActions):
            successor = gameState.generateSuccessor(index, action)
            temp = self.value(successor, 1, 0, a, b)
            if max(temp, val) == temp:
                val = temp
                currentAction = action
            a = max(val, a)
        return currentAction

    def value(self, state, index, depth, alpha, beta):
        if state.isWin() or state.isLose():
            return self.evaluationFunction(state)
        if depth == self.depth:
            return self.evaluationFunction(state)
        if index == 0:
            return self.max_value(state, index, depth, alpha, beta)
        else: 
            return self.min_value(state, index, depth, alpha, beta)

    def max_value(self, state, index, depth, alpha, beta):
        v = -(float("inf"))
        legalActions = state.getLegalActions(index)
        for action in list(legalActions):
            successor = state.generateSuccessor(index, action)
            temp = self.value(successor, 1, depth, alpha, beta)
            if v < temp:
                v = temp
            if v > beta:
                return v
            alpha = max(alpha, v)
        return v

    def min_value(self, state, index, depth, alpha, beta):
        v = float("inf")
        legalActions = state.getLegalActions(index)
        for action in list(legalActions):
            successor = state.generateSuccessor(index, action)
            ghosts = state.getNumAgents() - 1
            if index == ghosts:
                temp = self.value(successor, 0, depth + 1, alpha, beta)
            else: 
                temp = self.value(successor, index + 1, depth, alpha, beta)
            if temp < v:
                v = temp
            if v < alpha:
                return v
            beta = min(beta, v)
        return v 


class ExpectimaxAgent(MultiAgentSearchAgent):
    """
      Your expectimax agent (question 4)
    """

    def getAction(self, gameState):
        """
        Returns the expectimax action using self.depth and self.evaluationFunction

        All ghosts should be modeled as choosing uniformly at random from their
        legal moves.
        """
        "*** YOUR CODE HERE ***"
        """
        pseudocode 
        def value(state):
            if the state is terminal state: return the state's utility
            if the next agent is MAX: return max-value(state)
            if the next agent is MIN: return min-value(state)

        def max-value(state):
            initialize v = - inf
            for each sucessor of state:
                v = max(v, value(successor))
            return v

        def exp-value(state):
            initialize v = 0
            for each successor of state:
                p = probability(successor)
                v += p*value(successor)
            return v
        """
        index = 0
        numAgents = gameState.getNumAgents()
        legalActions = gameState.getLegalActions(index)
        val = -float("inf")
        currentAction = None
        for action in list(legalActions):
            successor = gameState.generateSuccessor(index, action)
            temp = self.value(successor, 1, 0)
            if max(temp, val) == temp:
                val = temp
                currentAction = action
        return currentAction

    def value(self, state, index, depth):
        if state.isWin() or state.isLose():
            return self.evaluationFunction(state)
        if depth == self.depth:
            return self.evaluationFunction(state)
        if index == 0:
            return self.max_value(state, index, depth)
        else: 
            return self.exp_value(state, index, depth)

    def max_value(self, state, index, depth):
        v = -(float("inf"))
        legalActions = state.getLegalActions(index)
        for action in list(legalActions):
            successor = state.generateSuccessor(index, action)
            temp = self.value(successor, 1, depth)
            if temp > v:
                v = temp
        return v

    def exp_value(self, state, index, depth):
        v = float("inf")
        legalActions = state.getLegalActions(index)
        exp = 0
        for action in list(legalActions):
            successor = state.generateSuccessor(index, action)
            ghosts = state.getNumAgents() - 1
            if index == ghosts:
                temp = self.value(successor, 0, depth + 1)
                if temp < v:
                    exp += temp
                else:
                    exp += v
            else: 
                temp = self.value(successor, index + 1, depth)
                if temp < v:
                    exp += temp
                else: 
                    exp += v
        return exp/len(state.getLegalActions(index))

def betterEvaluationFunction(currentGameState):
    """
    Your extreme ghost-hunting, pellet-nabbing, food-gobbling, unstoppable
    evaluation function (question 5).

    DESCRIPTION: <write something here so we know what you did>
    """
    "*** YOUR CODE HERE ***"

    newPos = currentGameState.getPacmanPosition()
    newFood = currentGameState.getFood()
    newGhostStates = currentGameState.getGhostStates()
    newScaredTimes = [ghostState.scaredTimer for ghostState in newGhostStates]    

    foodDist = 0
    ghostDist = 0
    if currentGameState.isLose():
        return -float("inf")
    elif currentGameState.isWin():
        return float("inf")

    for food in list(newFood.asList()):
        foodDist += util.manhattanDistance(food, newPos)

    for ghost in newGhostStates:
        if ghost.scaredTimer == 1:
            ghostDist += 2 * util.manhattanDistance(ghost.getPosition(), newPos)
        else:
            ghostDist += util.manhattanDistance(ghost.getPosition(), newPos)

    return currentGameState.getScore() + (1/foodDist) + ghostDist
    

# Abbreviation
better = betterEvaluationFunction
















