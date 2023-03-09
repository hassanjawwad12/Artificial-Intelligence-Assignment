 #implementing priority queues where the queue item with higher weight is given more priority in processing
import heapq

# This is the manhattan distance heuristic function and it takes the current stat and the state where it wants to go 
def heuristic1(state, goal_state, n):

    distance = 0
    for i in range(n):
        for j in range(n):
            value = state[n*i+j]
            if value != 0:
                row = (value-1) // n
                col = (value-1) % n
                #we have used abs here to return the absolute value of a specified number 
                distance += abs(row-i) + abs(col-j) #This is bascially the euclidean distance 
    return distance

#heuristic function to calulate the number of misplaced tiles 
def heuristic2(state, goal_state, n):
    
    misplaced = 0
    for i in range(n*n):
        if state[i] != goal_state[i]:
            misplaced += 1
    return misplaced
#it simply increments the number of misplaced tiles if the current state is not equal to the goal state and returns the value

def UCS(initial_state, goal_state):
    n = int(len(initial_state) ** 0.5)
    visited = set()
    queue = [(0, 0, initial_state, [])]
    while queue:
        cost, _, state, path = heapq.heappop(queue)
        if state == goal_state:
            return True, path + [state]
        visited.add(tuple(state))
        for neighbor in getNeighbours(state, n):
            if tuple(neighbor) not in visited:
                heapq.heappush(queue, (cost+1, 0, neighbor, path + [state]))
    return False, []

def greedyBFS(initial_state, goal_state, heuristic):
    n = int(len(initial_state) ** 0.5)
    visited = set()
    queue = [(heuristic(initial_state, goal_state, n), initial_state, [])]
    while queue:
        _, state, path = heapq.heappop(queue)
        if state == goal_state:
            return True, path + [state]
        visited.add(tuple(state))
        for neighbor in getNeighbours(state, n):
            if tuple(neighbor) not in visited:
                heapq.heappush(queue, (heuristic(neighbor, goal_state, n), neighbor, path + [state]))
    return False, []

def aStarSearch(initial_state, goal_state, heuristic):
    n = int(len(initial_state) ** 0.5)
    visited = set()
    queue = [(heuristic(initial_state, goal_state, n), 0, initial_state, [])]
    while queue:
        _, cost, state, path = heapq.heappop(queue)
        if state == goal_state:
            return True, path + [state]
        visited.add(tuple(state))
        for neighbor in getNeighbours(state, n):
            if tuple(neighbor) not in visited:
                heapq.heappush(queue, (cost + heuristic(neighbor, goal_state, n), cost+1, neighbor, path + [state]))
    return False, []


# Define the function to get the neighbors of a state
def getNeighbours(state, n):
    """Get the neighbors of a state."""
    neighbors = []
    index = state.index(0)
    if index % n > 0:
        left = state[:]
        left[index], left[index-1] = left[index-1], left[index]
        neighbors.append(left)
    if index % n < n-1:
        right = state[:]
        right[index], right[index+1] = right[index+1], right[index]
        neighbors.append(right)
    if index // n > 0:
        up = state[:]
        up[index], up[index-n] = up[index-n], up[index]
        neighbors.append(up)
    if index // n < n-1:
        down = state[:]
        down[index], down[index+n] = down[index+n], down[index]
        neighbors.append(down)
    return neighbors

# Get the initial and goal state from the user
initial_state = list(map(int, input("Enter the initial state of the puzzle: ").split()))
goal_state = list(map(int, input("Enter the goal state of the puzzle: ").split()))
'''
print("/n Press 1 if you want to perform the unifrom cost search on the puzzle")
print("/n Press 2 if you want to perform the greedy best first search on the puzzle")
print("/n Press 3 if you want to perform the A* search on the puzzle")
'''
print("\nFirstlty we will perform the traversal using Uniform Cost Search!")
found, path = UCS(initial_state, goal_state)
if found:
    print("Goal state is reachable from the initial state.")
    print("Traversal:")
    for state in path:
        print(state)
else:
    print("Goal state is not reachable from the initial state.")
    
print("Solution path:")
for state in path:
    print(state)
print("\n")

print("Secondly we will perform the traversal using Best First Search!")
print("\n")

print("Using the Manhattan distance heuristic:")
found, path = greedyBFS(initial_state, goal_state, heuristic1)
if found:
    print("Goal state is reachable from the initial state.")
    print("Traversal:")
    for state in path:
        print(state)
else:
    print("Goal state is not reachable from the initial state.")

print("\n Using the number of misplaced tiles heuristic:")
found, path = greedyBFS(initial_state, goal_state, heuristic2)
if found:
    print("Goal state is reachable from the initial state.")
    print("Traversal:")
    for state in path:
        print(state)
else:
    print("Goal state is not reachable from the initial state.")
    
print("\nSolution path:")
for state in path:
    print(state)

print("\nThirdly we will perform the traversal using A*")
print("\n")

print("Using the Manhattan distance heuristic:")
found, path = aStarSearch(initial_state, goal_state, heuristic1)
if found:
    print("Goal state is reachable from the initial state.")
    print("Traversal:")
    for state in path:
        print(state)
else:
    print("Goal state is not reachable from the initial state.")

print("\nUsing the number of misplaced tiles heuristic:")
found, path = aStarSearch(initial_state, goal_state, heuristic2)
if found:
    print("Goal state is reachable from the initial state.")
    print("Traversal:")
    for state in path:
        print(state)
else:
    print("Goal state is not reachable from the initial state.")
    
print("Solution path:")
for state in path:
    print(state)