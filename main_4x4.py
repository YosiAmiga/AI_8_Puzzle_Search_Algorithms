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


# Global variables
GoalState = [1, 2, 3, 4,
             5, 6, 7, 8,
             9, 10, 11, 12,
             13, 14, 15, 16]
GoalNode = None  # at finding solution
NodesExpanded = 0  # total nodes visited
MaxSearchDeep = 0  # max deep

# ######### Iterative deepening uses DFS until reaching a given limit depth #########
def iterativeDeepening(startState, maxDepth):
    # Repeatedly depth-limit search till the
    # maximum depth
    for i in range(maxDepth):
        DFS(startState,i)


#########  DFS until reaching a given limit depth #########
def DFS(startState,limit_Depth):
    global  GoalNode
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
    boardVisited= set()
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
        node1 = node1 + " " +str(poss)
    # calculate Heuristic and set initial node

    key = Heuristic(node1.split())
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
        # print('in queue while loop line 107')
        # Pop the state with the smallest value in the queue and check:
        node = Queue.pop(0)
        # if the given state is the goal state, return the current queue -> it represents the route!
        if node.state == GoalState:
            GoalNode = node
            return
        """not the goal state, 
        call the successor function, subNode to check all possible paths: in 3x3 there will be 12, in 4x4 there's 24"""
        possiblePaths = successorFunction(node)
        # iterate on all the paths, and and choose the best one
        for path in possiblePaths:
            # convert each path(state) to map and copy all the values
            thisPath = path.map[:]
            # check if the current path was not visited before
            if thisPath not in boardVisited:
                # use Heuristic function, return the given path total estimation, the key will be it + the path depth.
                # Make the state to heuristic a list of strings representing each distance
                toHeuristic = []
                for i in path.state:
                    toHeuristic.append(str(i))
                key = Heuristic(toHeuristic)
                path.key = key + path.depth
                # add current path to the queue, and to the visited set
                Queue.append(path)
                boardVisited.add(path.map[:])



# The Manhattan distance of each goal from its goal position
slot_1_distance_from_goal_4x4 = [0, 1, 2, 3,
                                 1, 2, 3, 4,
                                 2, 3, 4, 5,
                                 3, 4, 5, 6]
slot_2_distance_from_goal_4x4 = [1, 0, 1, 2,
                                 2, 1, 2, 3,
                                 3, 2, 3, 4,
                                 4, 3, 4, 5]
slot_3_distance_from_goal_4x4 = [2, 1, 0, 1,
                                 3, 2, 1, 2,
                                 4, 3, 2, 3,
                                 5, 4, 3, 4]
slot_4_distance_from_goal_4x4 = [3, 2, 1, 0,
                                 4, 3, 2, 1,
                                 5, 4, 3, 2,
                                 6, 5, 4, 3]
slot_5_distance_from_goal_4x4 = [1, 2, 3, 4,
                                 0, 1, 2, 3,
                                 1, 2, 3, 4,
                                 2, 3, 4, 5]
slot_6_distance_from_goal_4x4 = [2, 1, 2, 3,
                                 1, 0, 1, 2,
                                 2, 1, 2, 3,
                                 3, 2, 3, 4]
slot_7_distance_from_goal_4x4 = [3, 2, 1, 2,
                                 2, 1, 0, 1,
                                 3, 2, 1, 2,
                                 4, 3, 2, 3]
slot_8_distance_from_goal_4x4 = [4, 3, 2, 1,
                                 3, 2, 1, 0,
                                 4, 3, 2, 1,
                                 5, 4, 3, 2]
slot_9_distance_from_goal_4x4 = [2, 3, 4, 5,
                                 1, 2, 3, 4,
                                 0, 1, 2, 3,
                                 1, 2, 3, 4]
slot_10_distance_from_goal_4x4 = [3, 2, 3, 4,
                                  2, 1, 2, 3,
                                  1, 0, 1, 2,
                                  2, 1, 2, 3]
slot_11_distance_from_goal_4x4 = [4, 3, 2, 3,
                                  3, 2, 1, 2,
                                  2, 1, 0, 1,
                                  3, 2, 1, 2]
slot_12_distance_from_goal_4x4 = [5, 4, 3, 2,
                                  4, 3, 2, 1,
                                  3, 2, 1, 0,
                                  2, 3, 2, 1]
slot_13_distance_from_goal_4x4 = [3, 4, 5, 6,
                                  2, 3, 4, 5,
                                  1, 2, 3, 4,
                                  0, 1, 2, 3]
slot_14_distance_from_goal_4x4 = [4, 3, 4, 5,
                                  3, 2, 3, 4,
                                  2, 1, 2, 3,
                                  1, 0, 1, 2]
slot_15_distance_from_goal_4x4 = [5, 4, 3, 4,
                                  4, 3, 2, 3,
                                  3, 2, 1, 2,
                                  2, 1, 0, 1]
slot_16_distance_from_goal_4x4 = [6, 5, 4, 3,
                                  5, 4, 3, 2,
                                  4, 3, 2, 1,
                                  3, 2, 1, 0]

"""
    The function to calculate each state heuristic to goal according to the distance of each number from its
    goal position.
"""


def Heuristic(node):
    #For 4x4
    global slot_1_distance_from_goal_4x4, slot_2_distance_from_goal_4x4, slot_3_distance_from_goal_4x4, slot_4_distance_from_goal_4x4
    global slot_5_distance_from_goal_4x4, sslot_6_distance_from_goal, slot_7_distance_from_goal_4x4, slot_8_distance_from_goal_4x4, slot_9_distance_from_goal_4x4, slot_10_distance_from_goal_4x4
    global slot_11_distance_from_goal_4x4, slot_12_distance_from_goal_4x4, slot_13_distance_from_goal_4x4, slot_14_distance_from_goal_4x4, slot_15_distance_from_goal_4x4, slot_16_distance_from_goal_4x4
    """ node.index = returns the value of each number in the current node,
     the value in the node represents the distance of the given index from its goal place 

     slot_0_distance_from_goal = go to each number's list, and return its current value from the goal   
     """
    v1 = slot_1_distance_from_goal_4x4[node.index("1")]
    v2 = slot_2_distance_from_goal_4x4[node.index("2")]
    v3 = slot_3_distance_from_goal_4x4[node.index("3")]
    v4 = slot_4_distance_from_goal_4x4[node.index("4")]
    v5 = slot_5_distance_from_goal_4x4[node.index("5")]
    v6 = slot_6_distance_from_goal_4x4[node.index("6")]
    v7 = slot_7_distance_from_goal_4x4[node.index("7")]
    v8 = slot_8_distance_from_goal_4x4[node.index("8")]
    v9 = slot_9_distance_from_goal_4x4[node.index("9")]
    v10 = slot_10_distance_from_goal_4x4[node.index("10")]
    v11 = slot_11_distance_from_goal_4x4[node.index("11")]
    v12 = slot_12_distance_from_goal_4x4[node.index("12")]
    v13 = slot_13_distance_from_goal_4x4[node.index("13")]
    v14 = slot_14_distance_from_goal_4x4[node.index("14")]
    v15 = slot_15_distance_from_goal_4x4[node.index("15")]
    v16 = slot_16_distance_from_goal_4x4[node.index("16")]

    valorTotal = v1 + v2 + v3 + v4 + v5 + v6 + v7 + v8 + v9 + v10 + v11 + v12 + v13 + v14 + v15 + v16
    # Sum all the values of each number -> that is the total value to be given to the node for heuristic estimation
    # The final heuristic =
    # int value of the sum of all the Manhattan distances of each tile from its goal position, divided by 2
    # int((v1+v2..+v16)/2). This is an admissible heuristic
    valorTotal = int(valorTotal / 2)
    return valorTotal


# Obtain Sub Nodes********************************************************
# THE successor function: returns for each node its possible paths
def successorFunction(node):
    global NodesExpanded
    # Add to the counter of nodes that were expanded
    NodesExpanded = NodesExpanded + 1
    # Create all possible states of the given state
    # for a 4x4 game there are 24 possible state from each given state to another
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
    nextPaths.append(PuzzleState(move(node.state, 13), node, 1, node.depth + 1, node.cost + 1, 0))
    nextPaths.append(PuzzleState(move(node.state, 14), node, 2, node.depth + 1, node.cost + 1, 0))
    nextPaths.append(PuzzleState(move(node.state, 15), node, 3, node.depth + 1, node.cost + 1, 0))
    nextPaths.append(PuzzleState(move(node.state, 16), node, 4, node.depth + 1, node.cost + 1, 0))
    nextPaths.append(PuzzleState(move(node.state, 17), node, 5, node.depth + 1, node.cost + 1, 0))
    nextPaths.append(PuzzleState(move(node.state, 18), node, 6, node.depth + 1, node.cost + 1, 0))
    nextPaths.append(PuzzleState(move(node.state, 19), node, 7, node.depth + 1, node.cost + 1, 0))
    nextPaths.append(PuzzleState(move(node.state, 20), node, 8, node.depth + 1, node.cost + 1, 0))
    nextPaths.append(PuzzleState(move(node.state, 21), node, 9, node.depth + 1, node.cost + 1, 0))
    nextPaths.append(PuzzleState(move(node.state, 22), node, 10, node.depth + 1, node.cost + 1, 0))
    nextPaths.append(PuzzleState(move(node.state, 23), node, 11, node.depth + 1, node.cost + 1, 0))
    nextPaths.append(PuzzleState(move(node.state, 24), node, 11, node.depth + 1, node.cost + 1, 0))
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
            moves in 4x4:
            1. (1,2)
            2. (1,5)
            3. (2,3)
            4. (2,6)
            5. (3,4)
            6. (3,7)
            7. (4,8)
            8. (5,6)
            9. (5,9)
            10. (6,10)
            11. (6,7)
            12. (7,8)
            13. (7,11)
            14. (8,12)        
            15. (9,10)
            16. (9,13)
            17. (10,11)
            18. (10,14)
            19. (11,12)
            20. (11,15)
            21. (12,16)
            22. (13,14)
            23. (14,15)
            24. (15,,16)
            """
    # Check all possible moves for the 4x4 board
    if (direction == 1):
        newState[0], newState[1] = state[1], state[0]
        return newState
    if (direction == 2):
        newState[0], newState[4] = state[4], state[0]
        return newState
    if (direction == 3):
        newState[1], newState[2] = state[2], state[1]
        return newState
    if (direction == 4):
        newState[1], newState[5] = state[5], state[1]
        return newState
    if (direction == 5):
        newState[2], newState[3] = state[3], state[2]
        return newState
    if (direction == 6):
        newState[2], newState[6] = state[6], state[2]
        return newState
    if (direction == 7):
        newState[3], newState[7] = state[7], state[3]
        return newState
    if (direction == 8):
        newState[4], newState[5] = state[5], state[4]
        return newState
    if (direction == 9):
        newState[4], newState[8] = state[8], state[4]
        return newState
    if (direction == 10):
        newState[5], newState[9] = state[9], state[5]
        return newState
    if (direction == 11):
        newState[5], newState[6] = state[6], state[5]
        return newState
    if (direction == 12):
        newState[6], newState[7] = state[7], state[6]
        return newState
    if (direction == 13):
        newState[6], newState[10] = state[10], state[6]
        return newState
    if (direction == 14):
        newState[7], newState[11] = state[11], state[7]
        return newState
    if (direction == 15):
        newState[8], newState[9] = state[9], state[8]
        return newState
    if (direction == 16):
        newState[8], newState[12] = state[12], state[8]
        return newState
    if (direction == 17):
        newState[9], newState[10] = state[10], state[9]
        return newState
    if (direction == 18):
        newState[9], newState[13] = state[13], state[9]
        return newState
    if (direction == 19):
        newState[10], newState[11] = state[11], state[10]
        return newState
    if (direction == 20):
        newState[10], newState[14] = state[14], state[10]
        return newState
    if (direction == 21):
        newState[11], newState[15] = state[15], state[11]
        return newState
    if (direction == 22):
        newState[12], newState[13] = state[13], state[12]
        return newState
    if (direction == 23):
        newState[13], newState[14] = state[14], state[13]
        return newState
    if (direction == 24):
        newState[14], newState[15] = state[15], state[14]
        return newState



# MAIN**************************************************************
def main():
    global GoalNode
    # Obtain information from calling parameters
    parser = argparse.ArgumentParser()
    parser.add_argument('method')
    # parser.add_argument('initialBoard')
    args = parser.parse_args()
    # data1 = args.initialBoard.split(",")
    with open('4x4_3.txt', 'r') as f:
        l = [[int(num) for num in line.split(' ')] for line in f]

    list = []
    for i in l:
        for j in i:
            list.append(str(j))

    data = list
    print(data)
    """append all 16 cell values in the board"""
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
    InitialState.append(int(data[9]))
    InitialState.append(int(data[10]))
    InitialState.append(int(data[11]))
    InitialState.append(int(data[12]))
    InitialState.append(int(data[13]))
    InitialState.append(int(data[14]))
    InitialState.append(int(data[15]))

    # Start operation
    # calculate the running time
    start = timeit.default_timer()

    # Choose your search algorithm
    function = args.method
    if (function == "bfs"):
        bfs(InitialState)
    # if (function == "id"):
    #     iterativeDeepening(InitialState, 10)
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
    moves in 4x4:
    1.(1,2)
    2.(1,5)
    3.(2,3)
    4.(2,6)
    5.(3,4)
    6.(3,7)
    7.(4,8)
    8.(5,9)
    9.(5,6)
    10.(6,10)
    11.(6,10)
    12.(6,7)
    13. (7,8)
    14.(7,11)
    15. (8,12)        
    16. (9,10)
    17. (9,13)
    18. (10,11)
    19. (10,14)
    20. (11,12)
    21. (11,15)
    22.(12,16)
    23. (13,14)
    24. (15,,16)
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
        if GoalNode.move == 13:
            path = str(GoalNode.parent)
        if GoalNode.move == 14:
            path = str(GoalNode.parent)
        if GoalNode.move == 15:
            path = str(GoalNode.parent)
        if GoalNode.move == 16:
            path = str(GoalNode.parent)
        if GoalNode.move == 17:
            path = str(GoalNode.parent)
        if GoalNode.move == 18:
            path = str(GoalNode.parent)
        if GoalNode.move == 19:
            path = str(GoalNode.parent)
        if GoalNode.move == 20:
            path = str(GoalNode.parent)
        if GoalNode.move == 22:
            path = str(GoalNode.parent)
        if GoalNode.move == 23:
            path = str(GoalNode.parent)
        if GoalNode.move == 24:
            path = str(GoalNode.parent)
        moves.insert(0, path)
        GoalNode = GoalNode.parent

        # Print results

    print('Function selected:',function)
    print("Path to goal: ",moves)
    print("Cost: ", len(moves))
    print("Nodes expanded: ", str(NodesExpanded))
    print("Search depth: ", str(deep))
    print("Running time: ", format(time, '.8f'))

    # Generate output document for grade system
    file = open('output_4x4.txt', 'w')

    file.write("Path to goal: " + str(moves) + "\n")
    file.write("Cost : " + str(len(moves)) + "\n")
    file.write("Nodes expanded: " + str(NodesExpanded) + "\n")
    file.write("Search depth: " + str(deep) + "\n")
    file.write("Running time: " + format(time, '.8f') + "\n")
    file.close()




if __name__ == '__main__':
    main()
