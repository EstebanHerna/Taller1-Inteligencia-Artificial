from algorithms.problems import SearchProblem
import algorithms.utils as utils
from world.game import Directions
from algorithms.heuristics import nullHeuristic


def tinyHouseSearch(problem: SearchProblem):
    """
    Returns a sequence of moves that solves tinyHouse. For any other building, the
    sequence of moves will be incorrect, so only use this for tinyHouse.
    """
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
    # Pila LIFO para DFS (Last In, First Out)
    frontier = utils.Stack()
    
    # Nodo inicial: (estado, [acciones para llegar])
    start_state = problem.getStartState()
    frontier.push((start_state, []))
    
    visited = set()
    
    while not frontier.isEmpty():
        # Extraer el nodo más reciente (profundidad)
        current_state, actions = frontier.pop()
        
        # Si ya visitamos este estado, saltar
        if current_state in visited:
            continue
            
        # Marcar como visitado
        visited.add(current_state)
        
        # Verificar si es objetivo
        if problem.isGoalState(current_state):
            return actions
        
        # Expandir sucesores
        for successor, action, stepCost in problem.getSuccessors(current_state):
            if successor not in visited:
                # Agregar nuevo camino a la pila
                new_actions = actions + [action]
                frontier.push((successor, new_actions))
    
    return []


def breadthFirstSearch(problem: SearchProblem):
    """
    Search the shallowest nodes in the search tree first.
    
    BFS garantiza encontrar el camino con menor número de pasos.
    """
    # Cola FIFO para BFS (First In, First Out)
    frontier = utils.Queue()
    
    # Nodo inicial
    start_state = problem.getStartState()
    frontier.push((start_state, []))
    
    # Set de visitados
    visited = set()
    visited.add(start_state)  # Marcar inicio como visitado inmediatamente
    
    while not frontier.isEmpty():
        # Extraer el nodo más antiguo (amplitud)
        current_state, actions = frontier.pop()
        
        # Verificar si es objetivo
        if problem.isGoalState(current_state):
            return actions
        
        # Expandir sucesores
        for successor, action, stepCost in problem.getSuccessors(current_state):
            if successor not in visited:
                visited.add(successor)  # Marcar como visitado AL AGREGAR
                new_actions = actions + [action]
                frontier.push((successor, new_actions))
    
    return []


def uniformCostSearch(problem: SearchProblem):
    """
    Search the node of least total cost first.
    
    UCS expande nodos en orden de costo acumulado más bajo.
    """
    # Cola de prioridad (min-heap por costo)
    frontier = utils.PriorityQueue()
    
    # Nodo inicial: (estado, acciones, costo_acumulado)
    start_state = problem.getStartState()
    frontier.push((start_state, [], 0), 0)  # prioridad = costo
    
    # Diccionario de visitados con sus costos mínimos
    visited = {}
    
    while not frontier.isEmpty():
        # Extraer nodo con menor costo acumulado
        current_state, actions, current_cost = frontier.pop()
        
        # Si ya encontramos un camino mejor a este estado, saltar
        if current_state in visited:
            continue
            
        # Marcar como visitado con su costo
        visited[current_state] = current_cost
        
        
        if problem.isGoalState(current_state):
            return actions
        
        # Expandir sucesores
        for successor, action, stepCost in problem.getSuccessors(current_state):
            if successor not in visited:
                new_actions = actions + [action]
                new_cost = current_cost + stepCost
                frontier.push((successor, new_actions, new_cost), new_cost)
    
    return []


def aStarSearch(problem: SearchProblem, heuristic=nullHeuristic):
    """
    Search the node that has the lowest combined cost and heuristic first.
    
    A* usa: f(n) = g(n) + h(n)
    donde g(n) es el costo real y h(n) es la estimación heurística.
    """
    # Cola de prioridad ordenada por f(n) = g(n) + h(n)
    frontier = utils.PriorityQueue()
    
    # Nodo inicial
    start_state = problem.getStartState()
    h_start = heuristic(start_state, problem)
    frontier.push((start_state, [], 0), h_start)  # prioridad inicial = h(start)
    
    # Diccionario de visitados
    visited = {}
    
    while not frontier.isEmpty():
        # Extraer nodo con menor f(n)
        current_state, actions, g_cost = frontier.pop()
        
        # Si ya visitamos con mejor costo, saltar
        if current_state in visited:
            continue
            
        # Marcar como visitado
        visited[current_state] = g_cost
        
        # Verificar si es objetivo
        if problem.isGoalState(current_state):
            return actions
        
        # Expandir sucesores
        for successor, action, stepCost in problem.getSuccessors(current_state):
            if successor not in visited:
                new_actions = actions + [action]
                new_g_cost = g_cost + stepCost
                # f(n) = g(n) + h(n)
                h_cost = heuristic(successor, problem)
                f_cost = new_g_cost + h_cost
                frontier.push((successor, new_actions, new_g_cost), f_cost)
    
    return []


bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch