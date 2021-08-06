import search
from AirSimProblem import airsimproblem
from AirSumRunPath import airsumrunpath
from utils import hashabledict
import numpy as np
import time
import pickle
from MctsAirSimProblem import MonteCarloTreeSearchNode

np.random.seed(0)


def main():
    num_drones = 5
    num_packages = 5
    max_dist = 1e+6
    initial = hashabledict()
    initial['drones'] = hashabledict({i + 1: 0 for i in range(num_drones)})
    initial['packages'] = hashabledict({i + 1: 0 for i in range(num_packages)})

    goal = hashabledict()
    goal['drones'] = hashabledict({i + 1: 0 for i in range(num_drones)})
    goal['packages'] = hashabledict({i + 1: -1 for i in range(num_packages)})

    packages_locations = np.random.randint(-100, 100, size=(num_packages, 2))  # generating 20 coordinates, ignoring z - assuming on land

    asprob = airsimproblem(max_dist, initial, goal, packages_locations)
    now = time.time()
    goal_node = search.astar_search(asprob)
    print(f'done in {time.time() - now:.2f}s\n')

    solution_path = goal_node.path()
    print(f"goal:\n {solution_path}")
    with open('solution.pkl', 'wb') as f:
        pickle.dump(solution_path, f)

    asrunpath = airsumrunpath(num_drones, packages_locations)
    asrunpath.follow_path(solution_path)


if __name__ == "__main__":
    num_drones = 5
    num_packages = 5
    max_dist = 1e+6
    initial = hashabledict()
    initial['drones'] = hashabledict({i + 1: 0 for i in range(num_drones)})
    initial['packages'] = hashabledict({i + 1: 0 for i in range(num_packages)})

    goal = hashabledict()
    goal['drones'] = hashabledict({i + 1: 0 for i in range(num_drones)})
    goal['packages'] = hashabledict({i + 1: -1 for i in range(num_packages)})

    root = MonteCarloTreeSearchNode(state=initial, goal=goal)
    selected_action = root.best_action()
    print(selected_action)