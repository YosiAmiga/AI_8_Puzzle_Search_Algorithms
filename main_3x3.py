import argparse
import timeit
# Information *****************************************************
# Creating the Puzzle State class, in the constructor:
from collections import deque

"""
    State is the puzzle representation in a list.
    parent is the previous state we came from.
    move is the move we choose each time.
    depth is the depth of the route from start to goal.
    cost is the cost of each state switching.
    key is the calculated estimation of each state to goal.
"""


class PuzzleState:
    def __init__(self, state, parent, move, depth, cost, key):
        self.state = state
        self.parent = parent
        self.move = move
        self.depth = depth
        self.cost = cost
        self.key = key
        if self.state:
            self.map = ''.join(str(e) for e in self.state)

    # create functions in order to make states comparable
    def __eq__(self, other):
        return self.map == other.map

    def __lt__(self, other):
        return self.map < other.map

    def __str__(self):
        return str(self.map)


# Global variables ***********************************************



GoalState = [1, 2, 3, 4, 5, 6, 7, 8, 9]
GoalNode = None  # at finding solution
NodesExpanded = 0  # total nodes visited
MaxSearchDeep = 0  # max deep
MaxFrontier = 0  # max frontier



# ######### Iterative deepening uses DFS until reaching a given limit depth #########
def iterativeDeepening(startState, maxDepth):
    # Repeatedly depth-limit search till the
    # maximum depth
    for i in range(maxDepth):
        DFS(startState,i)


#########  DFS until reaching a given limit depth #########
def DFS(startState,limit_Depth):
    global GoalNode
    # Create a set of visited states, and add to it in order to exclude state that already been visited
    # A stack in order to traverse the tree in a Depth First Search order: visit Parent node and then its children.
    boardVisited = set()
    stack = list([PuzzleState(startState, None, None, 0, 0, 0)])
    # While the stack is not empty, search for the goal state.
    while stack:
        # pop the head of the stack, and add to visited set, then check if its the goal
        node = stack.pop()
        print(node)
        boardVisited.add(node.map)
        # if its the goal state, return true to the ID method, and save the current node in order to find the route.
        if node.state == GoalState:
            GoalNode = node
            return True
        # call the successor function on the current node and return all the next states as possible paths.
        possiblePaths = successorFunction(node)
        for path in possiblePaths:
            if path.map not in boardVisited:
                # if the limit depth has not been surpassed, add the current path to the stack.
                if path.depth < limit_Depth:
                    stack.append(path)


#########BFS = A star with 0 in heuristic #########
def bfs(startState):

    global GoalNode
    # Create a set of visited states, and add to it in order to exclude state that already been visited
    # Create a queue in order to select the all the nodes to traverse on in the current height
    boardVisited = set()
    Queue = deque([PuzzleState(startState, None, None, 0, 0, 0)])
    # While the queue is not empty
    while Queue:
        # pop The "left" element in the queue, only pop one element, it will be the current node
        node = Queue.popleft()
        # add the current node to the visited set, so we won't search in it again
        boardVisited.add(node.map)
        # if we reached the goal state, set the GoalNode to be the current node, and stop the algorithm.
        if node.state == GoalState:
            # print('Queue length when we reached to goal: ', len(Queue))
            GoalNode = node
            return
        # if not, call the successor function and view all the possible paths from the current node
        possiblePaths = successorFunction(node)
        for path in possiblePaths:
            if path.map not in boardVisited:
                # we reached a state that was not visited, add it to the queue for further expanding on the next level.
                Queue.append(path)



######### AST & BFS, Bfs = A star with 0 in heuristic #########
def ast(startState):
    global GoalNode
    print("Start state is: " + str(startState))
    # transform initial state to calculate Heuristic, nod1 will be the start state as a list of numbers
    node1 = ""
    for poss in startState:
        node1 = node1 + str(poss)
    # calculate Heuristic and set initial node
    # CHECK IF BFS OR ASTAR

    key = Heuristic(node1)
    # Create a set of visited states, and add to it in order to exclude state that already been visited
    # Create a queue in order to select the best route from each state until we reach the goal state
    boardVisited = set()
    Queue = []
    # The first state in the queue will be the initial state that was given in the function
    # add in to the set of visited states
    Queue.append(PuzzleState(startState, None, None, 0, 0, key))
    boardVisited.add(node1)

    while Queue:
        # Sort all the state in the current queue by their heuristic - from the smallest value to the biggest
        Queue.sort(key=lambda o: o.key)
        # Pop the state with the smallest value in the queue and check:
        node = Queue.pop(0)
        # if the given state is the goal state, return the current queue -> it represents the route!
        if node.state == GoalState:
            GoalNode = node
            return
        """not the goal state, 
        call the successor function, subNode to check all possible paths: in 3x3 there will be 12, in 4x4 there's 24"""
        posiblePaths = successorFunction(node)
        # iterate on all the paths, and and choose the best one
        for path in posiblePaths:
            # convert each path(state) to map and copy all the values
            thisPath = path.map[:]
            # check if the current path was not visited before
            if thisPath not in boardVisited:
                # use Heuristic function, return the given path total estimation, the key will be it + the path depth.
                key = Heuristic(path.map)
                path.key = key + path.depth
                # add current path to the queue, and to the visited set
                Queue.append(path)
                boardVisited.add(path.map[:])



# The Manhattan distance of each goal from its goal position
slot_1_distance_from_goal = [0, 1, 2, 1, 2, 3, 2, 3, 4]
slot_2_distance_from_goal = [1, 0, 1, 2, 1, 2, 3, 2, 3]
slot_3_distance_from_goal = [2, 1, 0, 3, 2, 1, 4, 3, 2]
slot_4_distance_from_goal = [1, 2, 3, 0, 1, 2, 1, 2, 3]
slot_5_distance_from_goal = [2, 1, 2, 1, 0, 1, 2, 1, 2]
slot_6_distance_from_goal = [3, 2, 1, 2, 1, 0, 3, 2, 1]
slot_7_distance_from_goal = [2, 3, 4, 1, 2, 3, 0, 1, 2]
slot_8_distance_from_goal = [3, 2, 3, 2, 1, 2, 1, 0, 1]
slot_9_distance_from_goal = [4, 3, 2, 3, 2, 1, 2, 1, 0]

"""
    The function to calculate each state heuristic to goal according to the distance of each number from its
    goal position.
"""

def Heuristic(node):
    global slot_0_distance_from_goal, slot_1_distance_from_goal, slot_2_distance_from_goal, slot_3_distance_from_goal, slot_4_distance_from_goal, slot_5_distance_from_goal, slot_6_distance_from_goal, slot_7_distance_from_goal, slot_8_distance_from_goal
    """ node.index = returns the value of each number in the current node,
     the value in the node represents the distance of the given index from its goal place 

     slot_0_distance_from_goal = go to each number's list, and return its current value from the goal   
     """

    v0 = slot_1_distance_from_goal[node.index("1")]
    v1 = slot_2_distance_from_goal[node.index("2")]
    v2 = slot_3_distance_from_goal[node.index("3")]
    v3 = slot_4_distance_from_goal[node.index("4")]
    v4 = slot_5_distance_from_goal[node.index("5")]
    v5 = slot_6_distance_from_goal[node.index("6")]
    v6 = slot_7_distance_from_goal[node.index("7")]
    v7 = slot_8_distance_from_goal[node.index("8")]
    v8 = slot_9_distance_from_goal[node.index("9")]

    # Sum all the values of each number -> that is the total value to be given to the node for heuristic estimation
    # The final heuristic =
    # int value of the sum of all the Manhattan distances of each tile from its goal position, divided by 2
    # int((v0+v1..+v8)/2). This is an admissible heuristic
    valorTotal = v0 + v1 + v2 + v3 + v4 + v5 + v6 + v7 + v8
    valorTotal = int(valorTotal/2)
    return valorTotal



# THE successor function: returns for each node its possible paths
def successorFunction(node):
    global NodesExpanded
    # Add to the counter of nodes that were expanded
    NodesExpanded = NodesExpanded + 1
    # Create all possible states of the given state
    nextPaths = []
    nextPaths.append(PuzzleState(move(node.state, 1), node, 1, node.depth + 1, node.cost + 1, 0))
    nextPaths.append(PuzzleState(move(node.state, 2), node, 2, node.depth + 1, node.cost + 1, 0))
    nextPaths.append(PuzzleState(move(node.state, 3), node, 3, node.depth + 1, node.cost + 1, 0))
    nextPaths.append(PuzzleState(move(node.state, 4), node, 4, node.depth + 1, node.cost + 1, 0))
    nextPaths.append(PuzzleState(move(node.state, 5), node, 5, node.depth + 1, node.cost + 1, 0))
    nextPaths.append(PuzzleState(move(node.state, 6), node, 6, node.depth + 1, node.cost + 1, 0))
    nextPaths.append(PuzzleState(move(node.state, 7), node, 7, node.depth + 1, node.cost + 1, 0))
    nextPaths.append(PuzzleState(move(node.state, 8), node, 8, node.depth + 1, node.cost + 1, 0))
    nextPaths.append(PuzzleState(move(node.state, 9), node, 9, node.depth + 1, node.cost + 1, 0))
    nextPaths.append(PuzzleState(move(node.state, 10), node, 10, node.depth + 1, node.cost + 1, 0))
    nextPaths.append(PuzzleState(move(node.state, 11), node, 11, node.depth + 1, node.cost + 1, 0))
    nextPaths.append(PuzzleState(move(node.state, 12), node, 11, node.depth + 1, node.cost + 1, 0))

    #############################
    nodes = []
    for procPaths in nextPaths:
        if (procPaths.state != None):
            nodes.append(procPaths)
    return nodes


# Next step**************************************************************
def move(state, direction):
    # generate a copy
    newState = state[:]

    """
    directions in 3x3:
    1.(1,2)
    2.(1,4)
    3.(2,3)
    4.(2,5)
    5.(3,6)
    6.(5,4)
    7.(5,6)
    8.(7,4)
    9.(7,8)
    10.(8,5)
    11.(8,9)
    12.(9,6)
    """

    # obtain poss of 0
    if (direction == 1):
        newState[0], newState[1] = state[1], state[0]
        return newState
    if (direction == 2):
        newState[0], newState[3] = state[3], state[0]
        return newState
    if (direction == 3):
        newState[1], newState[2] = state[2], state[1]
        return newState
    if (direction == 4):
        newState[1], newState[4] = state[4], state[1]
        return newState
    if (direction == 5):
        newState[2], newState[5] = state[5], state[2]
        return newState
    if (direction == 6):
        newState[4], newState[3] = state[3], state[4]
        return newState
    if (direction == 7):
        newState[4], newState[5] = state[5], state[4]
        return newState
    if (direction == 8):
        newState[6], newState[3] = state[3], state[6]
        return newState
    if (direction == 9):
        newState[6], newState[7] = state[7], state[6]
        return newState
    if (direction == 10):
        newState[7], newState[4] = state[4], state[7]
        return newState
    if (direction == 11):
        newState[7], newState[8] = state[8], state[7]
        return newState
    if (direction == 12):
        newState[8], newState[5] = state[5], state[8]
        return newState
    #######################################################



####################### MAIN ################################
def main():
    global GoalNode
    # Obtain information from calling parameters
    parser = argparse.ArgumentParser()
    parser.add_argument('method')
    # parser.add_argument('initialBoard')
    args = parser.parse_args()
    # data1 = args.initialBoard.split(",")
    with open('3x3_2.txt', 'r') as f:
        l = [[int(num) for num in line.split(' ')] for line in f]

    list = []
    for i in l:
        for j in i:
            list.append(str(j))

    data = list
    print(data)

    # Build initial board state
    InitialState = []
    InitialState.append(int(data[0]))
    InitialState.append(int(data[1]))
    InitialState.append(int(data[2]))
    InitialState.append(int(data[3]))
    InitialState.append(int(data[4]))
    InitialState.append(int(data[5]))
    InitialState.append(int(data[6]))
    InitialState.append(int(data[7]))
    InitialState.append(int(data[8]))

    # Start operation
    # calculate the running time
    start = timeit.default_timer()

    # Choose your search algorithm
    function = args.method
    if (function == "bfs"):
        bfs(InitialState)
    if (function == "id"):
        iterativeDeepening(InitialState, 5)
    if (function == "astar"):
        ast(InitialState)

    stop = timeit.default_timer()
    time = stop - start

    # Save total path result
    deep = GoalNode.depth
    moves = []
    """
    moves in 3x3:
    1.(1,2)
    2.(1,4)
    3.(2,3)
    4.(2,5)
    5.(3,6)
    6.(5,4)
    7.(5,6)
    8.(7,4)
    9.(7,8)
    10.(8,5)
    11.(8,9)
    12.(9,6)
    """
    # iterate while we didn't reach the goal state
    while InitialState != GoalNode.state:
        if GoalNode.move == 1:
            path = str(GoalNode.parent)
        if GoalNode.move == 2:
            path = str(GoalNode.parent)
        if GoalNode.move == 3:
            path = str(GoalNode.parent)
        if GoalNode.move == 4:
            path = str(GoalNode.parent)
        if GoalNode.move == 5:
            path = str(GoalNode.parent)
        if GoalNode.move == 6:
            path = str(GoalNode.parent)
        if GoalNode.move == 7:
            path = str(GoalNode.parent)
        if GoalNode.move == 8:
            path = str(GoalNode.parent)
        if GoalNode.move == 9:
            path = str(GoalNode.parent)
        if GoalNode.move == 10:
            path = str(GoalNode.parent)
        if GoalNode.move == 11:
            path = str(GoalNode.parent)
        if GoalNode.move == 12:
            path = str(GoalNode.parent)

        moves.insert(0, path)
        GoalNode = GoalNode.parent

    # Print results
    print('Path to goal with function: ',function)
    print('\n')
    for m in moves:
        move_i = []
        firstRow = []
        secondRow = []
        thirdRow = []
        for i in range(9):
            if i < 3:
                firstRow.append(m[i])
            elif i >=3 and i < 6:
                secondRow.append(m[i])
            else:
                thirdRow.append(m[i])
        move_i.append(firstRow)
        move_i.append(secondRow)
        move_i.append(thirdRow)

        for m in move_i:
            print(m)
        print("\t")


    # print("path: ", moves)

    print("Cost: ", len(moves))
    print("Nodes expanded: ", str(NodesExpanded))
    print("Search depth: ", str(deep))
    print("Running time: ", format(time, '.8f'))

    # Generate output document for grade system
    # '''
    file = open('output_3x3.txt', 'w')

    file.write("Path to goal: " + str(moves) + "\n")
    file.write("Cost : " + str(len(moves)) + "\n")
    file.write("Nodes expanded: " + str(NodesExpanded) + "\n")
    file.write("Search depth: " + str(deep) + "\n")
    file.write("Running time: " + format(time, '.8f') + "\n")
    file.close()



if __name__ == '__main__':
    main()
