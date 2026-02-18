from typing import Any, Tuple
from algorithms import utils
from algorithms.problems import MultiSurvivorProblem


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0


def manhattanHeuristic(state, problem):
    """
    The Manhattan distance heuristic.
    
    La distancia Manhattan es la suma de las distancias absolutas
    en las coordenadas x e y: |x1-x2| + |y1-y2|
    
    Es admisible porque nunca sobreestima (el robot solo puede moverse
    en 4 direcciones, no en diagonal).
    """
    if isinstance(state, tuple) and len(state) == 2 and isinstance(state[0], (int, float)):
        # SimpleSurvivorProblem: state = (x, y)
        goal = problem.goal
        return abs(state[0] - goal[0]) + abs(state[1] - goal[1])
    else:
        # MultiSurvivorProblem: state = ((x, y), survivors_grid)
        # Este caso se maneja en survivorHeuristic
        return 0


def euclideanHeuristic(state, problem):
    """
    The Euclidean distance heuristic.
    
    La distancia Euclidiana es la distancia en línea recta:
    sqrt((x1-x2)² + (y1-y2)²)
    
    Es admisible porque es la distancia más corta posible entre dos puntos.
    Sin embargo, puede ser menos informativa que Manhattan para movimiento
    en grilla (4 direcciones).
    """
    import math
    
    # Para SimpleSurvivorProblem, state es una tupla (x, y)
    if isinstance(state, tuple) and len(state) == 2 and isinstance(state[0], (int, float)):
        goal = problem.goal
        dx = state[0] - goal[0]
        dy = state[1] - goal[1]
        return math.sqrt(dx * dx + dy * dy)
    else:
        # MultiSurvivorProblem
        return 0


def survivorHeuristic(state: Tuple[Tuple, Any], problem: MultiSurvivorProblem):
    """
    Your heuristic for the MultiSurvivorProblem.

    state: (position, survivors_grid)
    problem: MultiSurvivorProblem instance

    This must be admissible and preferably consistent.

    Hints:
    - Use problem.heuristicInfo to cache expensive computations
    - Go with some simple heuristics first, then build up to more complex ones
    - Consider: distance to nearest survivor + MST of remaining survivors
    - Balance heuristic strength vs. computation time (do experiments!)
    """
    position, survivors_grid = state
    
    # Si no quedan sobrevivientes, heurística = 0
    remaining_survivors = survivors_grid.asList()
    if len(remaining_survivors) == 0:
        return 0
    
    # ESTRATEGIA: Distancia Manhattan al sobreviviente más cercano
    # Esta es admisible porque:
    # - Nunca sobreestima (siempre hay que llegar al menos a un sobreviviente)
    # - Es optimista (asume que después de rescatar uno, los demás están en la misma posición)
    
    min_distance = float('inf')
    for survivor_pos in remaining_survivors:
        manhattan = abs(position[0] - survivor_pos[0]) + abs(position[1] - survivor_pos[1])
        if manhattan < min_distance:
            min_distance = manhattan
    
    # Heurística básica: solo distancia al más cercano
    # Para una heurística más fuerte (pero aún admisible), podrías agregar:
    # - Estimación del MST (Minimum Spanning Tree) de los sobrevivientes restantes
    # - Pero eso es más costoso computacionalmente
    
    return min_distance