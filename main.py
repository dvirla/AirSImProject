import search
from AirSimProblem import airsimproblem
from AirSumRunPath import airsumrunpath
from utils import hashabledict
import numpy as np
import time
import pickle
from MctsAirSimProblem import MonteCarloTreeSearchNode
from copy import copy
import matplotlib.pyplot as plt
import matplotlib.cm as cm

np.random.seed(0)


def main():
    num_drones = 5
    num_packages = 10
    max_dist = 1e+6
    initial = hashabledict()
    initial['drones'] = hashabledict({i + 1: 0 for i in range(num_drones)})
    initial['packages'] = hashabledict({i + 1: 0 for i in range(num_packages)})

    goal = hashabledict()
    goal['drones'] = hashabledict({i + 1: 0 for i in range(num_drones)})
    goal['packages'] = hashabledict({i + 1: -1 for i in range(num_packages)})

    packages_locations = np.random.randint(-100, 100, size=(
    num_packages, 2))  # generating 20 coordinates, ignoring z - assuming on land

    asprob = airsimproblem(max_dist, initial, goal, packages_locations)
    now = time.time()
    goal_node = search.astar_search(asprob)
    print(f'done in {time.time() - now:.2f}s\n')

    solution_path = goal_node.path()
    print(f"goal:\n {solution_path}")
    plt.scatter(packages_locations[:, 0], packages_locations[:, 1])
    colors = cm.rainbow(np.linspace(0, 1, num_drones))
    last_node = None
    total_dist = 0
    for node in solution_path:
        if last_node is None:
            last_node = node
            continue
        for drone in range(1, num_drones + 1):
            pos = np.array([0, 0])
            last_pos = np.array([0, 0])
            if node.state['drones'][drone] > 0:
                pos = packages_locations[node.state['drones'][drone] - 1]
            if last_node.state['drones'][drone] > 0:
                last_pos = packages_locations[last_node.state['drones'][drone] - 1]
            total_dist += np.sqrt(np.sum(np.square(pos - last_pos)))
            # if drone == 1:
            #     print(last_pos, pos)
            plt.plot([last_pos[0], pos[0]], [last_pos[1], pos[1]], color=colors[drone - 1])
        last_node = node
    print(total_dist)
    plt.show()
    with open('solution.pkl', 'wb') as f:
        pickle.dump(solution_path, f)

    # asrunpath = airsumrunpath(num_drones, packages_locations)
    # asrunpath.follow_path(solution_path)


def mcts():
    num_drones = 5
    num_packages = 20
    max_dist = 1e+6
    initial = hashabledict()
    initial['drones'] = hashabledict({i + 1: 0 for i in range(num_drones)})
    initial['packages'] = hashabledict({i + 1: 0 for i in range(num_packages)})

    goal = hashabledict()
    goal['drones'] = hashabledict({i + 1: 0 for i in range(num_drones)})
    goal['packages'] = hashabledict({i + 1: -1 for i in range(num_packages)})

    root = MonteCarloTreeSearchNode(state=initial, goal=goal)
    selected_action = copy(root)
    path = []
    total_dist = 0
    loc = MonteCarloTreeSearchNode.packages_locations
    plt.scatter(loc[:, 0], loc[:, 1])
    colors = cm.rainbow(np.linspace(0, 1, num_drones))
    while True:
        if selected_action.parent_action != None:
            for drone, destination in selected_action.parent_action.items():
                pos = np.array([0, 0])
                last_pos = np.array([0, 0])
                if destination > 0:
                    pos = loc[destination - 1]
                if selected_action.parent.state['drones'][drone] > 0:
                    last_pos = loc[selected_action.parent.state['drones'][drone] - 1]
                # if drone == 1:
                #     print(last_pos, pos)
                plt.plot([last_pos[0], pos[0]], [last_pos[1], pos[1]], color=colors[drone - 1])
            for drone, destinations in selected_action.state['drones'].items():
                drone_loc = np.array([0, 0])
                if selected_action.parent.state['drones'][drone] != 0:
                    drone_loc = MonteCarloTreeSearchNode.packages_locations[selected_action.parent.state['drones'][drone] - 1]
                if type(destinations) == int:
                    destinations = [destinations]
                for dest in destinations:
                    if dest > 0:
                        dest_loc = MonteCarloTreeSearchNode.packages_locations[dest - 1]
                        total_dist += np.linalg.norm(dest_loc - drone_loc)
                        drone_loc = dest_loc
        path.append(selected_action.state['drones'])
        if selected_action.is_terminal_node():
            break
        selected_action = selected_action.best_action()
    plt.show()

    print(total_dist)
    path.append(selected_action.state['drones'])
    print(path)
    MonteCarloTreeSearchNode.graph.visualize()


if __name__ == "__main__":
    mcts()
