import heapq
import copy
import random
import time
from collections import deque

# Funções auxiliares
def find_blank(state):
    for i in range(len(state)):
        for j in range(len(state[i])):
            if state[i][j] == 0:
                return i, j

def get_neighbors(state):
    neighbors = []
    i, j = find_blank(state)
    directions = [(i-1, j), (i+1, j), (i, j-1), (i, j+1)]
    for di, dj in directions:
        if 0 <= di < len(state) and 0 <= dj < len(state[0]):
            neighbor = copy.deepcopy(state)
            neighbor[i][j], neighbor[di][dj] = neighbor[di][dj], neighbor[i][j]
            neighbors.append(neighbor)
    return neighbors

def is_goal(state):
    return state == goal_state

# Funções de busca
def bfs(initial_state):
    start_time = time.time()
    frontier = deque([(initial_state, 0)])
    explored = set()
    nodes_explored = 0

    while frontier:
        state, depth = frontier.popleft()
        nodes_explored += 1
        if is_goal(state):
            end_time = time.time()
            return {
                'result': state,
                'time': end_time - start_time,
                'nodes_explored': nodes_explored,
                'depth': depth
            }
        explored.add(tuple(map(tuple, state)))
        for neighbor in get_neighbors(state):
            if tuple(map(tuple, neighbor)) not in explored:
                frontier.append((neighbor, depth + 1))
    return None

def dfs(initial_state):
    start_time = time.time()
    frontier = [(initial_state, 0)]
    explored = set()
    nodes_explored = 0

    while frontier:
        state, depth = frontier.pop()
        nodes_explored += 1
        if is_goal(state):
            end_time = time.time()
            return {
                'result': state,
                'time': end_time - start_time,
                'nodes_explored': nodes_explored,
                'depth': depth
            }
        explored.add(tuple(map(tuple, state)))
        for neighbor in get_neighbors(state):
            if tuple(map(tuple, neighbor)) not in explored:
                frontier.append((neighbor, depth + 1))
    return None

def uniform_cost_search(initial_state):
    start_time = time.time()
    frontier = [(0, initial_state, 0)]
    explored = set()
    nodes_explored = 0

    while frontier:
        cost, state, depth = heapq.heappop(frontier)
        nodes_explored += 1
        if is_goal(state):
            end_time = time.time()
            return {
                'result': state,
                'time': end_time - start_time,
                'nodes_explored': nodes_explored,
                'depth': depth
            }
        explored.add(tuple(map(tuple, state)))
        for neighbor in get_neighbors(state):
            if tuple(map(tuple, neighbor)) not in explored:
                heapq.heappush(frontier, (cost + 1, neighbor, depth + 1))
    return None

def heuristic_misplaced_tiles(state):
    count = 0
    for i in range(len(state)):
        for j in range(len(state[i])):
            if state[i][j] != goal_state[i][j] and state[i][j] != 0:
                count += 1
    return count

def heuristic_manhattan_distance(state):
    total_distance = 0
    for i in range(len(state)):
        for j in range(len(state[i])):
            if state[i][j] != 0:
                goal_i, goal_j = divmod(goal_state_flat.index(state[i][j]), 3)
                total_distance += abs(i - goal_i) + abs(j - goal_j)
    return total_distance

def greedy_search(initial_state):
    start_time = time.time()
    frontier = [(heuristic_misplaced_tiles(initial_state), initial_state, 0)]
    explored = set()
    nodes_explored = 0

    while frontier:
        h, state, depth = heapq.heappop(frontier)
        nodes_explored += 1
        if is_goal(state):
            end_time = time.time()
            return {
                'result': state,
                'time': end_time - start_time,
                'nodes_explored': nodes_explored,
                'depth': depth
            }
        explored.add(tuple(map(tuple, state)))
        for neighbor in get_neighbors(state):
            if tuple(map(tuple, neighbor)) not in explored:
                heapq.heappush(frontier, (heuristic_misplaced_tiles(neighbor), neighbor, depth + 1))
    return None

def a_star_search(initial_state, heuristic):
    start_time = time.time()

    frontier = [(heuristic(initial_state), 0, initial_state, 0)]
    explored = set()
    nodes_explored = 0

    while frontier:
        f, g, state, depth = heapq.heappop(frontier)
        nodes_explored += 1
        if is_goal(state):
            end_time = time.time()
            return {
                'result': state,
                'time': end_time - start_time,
                'nodes_explored': nodes_explored,
                'depth': depth
            }
        explored.add(tuple(map(tuple, state)))
        for neighbor in get_neighbors(state):
            if tuple(map(tuple, neighbor)) not in explored:
                new_g = g + 1
                new_f = new_g + heuristic(neighbor)
                heapq.heappush(frontier, (new_f, new_g, neighbor, depth + 1))
    return None

# Gera um estado inicial aleatório
def generate_random_initial_state():
    numbers = list(range(9))
    random.shuffle(numbers)
    initial_state = [numbers[i:i + 3] for i in range(0, 9, 3)]
    return initial_state

random_initial_state = generate_random_initial_state()

print("Random Initial State:")
for row in random_initial_state:
    print(row)

# Estado objetivo
goal_state = [[1, 2, 3],
              [4, 5, 6],
              [7, 8, 0]]

# Flattened goal state for Manhattan distance calculation
goal_state_flat = [item for sublist in goal_state for item in sublist]

# Testes
result_bfs = bfs(random_initial_state)
result_dfs = dfs(random_initial_state)
result_ucs = uniform_cost_search(random_initial_state)
result_greedy = greedy_search(random_initial_state)
result_a_star_h1 = a_star_search(random_initial_state, heuristic_misplaced_tiles)
result_a_star_h2 = a_star_search(random_initial_state, heuristic_manhattan_distance)

# Função para imprimir os resultados
def print_results(name, result):
    if result:
        print(f"\n{name} Result:")
        print("Time:", result['time'])
        print("Nodes Explored:", result['nodes_explored'])
        print("Solution Depth:", result['depth'])
        print("Solution Path:")
        for row in result['result']:
            print(row)
    else:
        print(f"\n{name} Result: No solution found.")

print_results("BFS", result_bfs)
print_results("DFS", result_dfs)
print_results("Uniform Cost Search", result_ucs)
print_results("Greedy Search", result_greedy)
print_results("A* with Misplaced Tiles Heuristic", result_a_star_h1)
print_results("A* with Manhattan Distance Heuristic", result_a_star_h2)
