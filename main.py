import search
from AirSimProblem import airsimproblem
# from AirSumRunPath import airsumrunpath
from utils import hashabledict
import numpy as np
import time
import pickle
from MctsAirSimProblem import MonteCarloTreeSearchNode
from copy import copy
import matplotlib.pyplot as plt
import matplotlib.cm as cm

np.random.seed(0)


def main(n_drones=5, n_packages=10, max_dist=1e+6):
    initial = hashabledict()
    initial['drones'] = hashabledict({i + 1: 0 for i in range(n_drones)})
    initial['packages'] = hashabledict({i + 1: 0 for i in range(n_packages)})

    goal = hashabledict()
    goal['drones'] = hashabledict({i + 1: 0 for i in range(n_drones)})
    goal['packages'] = hashabledict({i + 1: -1 for i in range(n_packages)})

    packages_locations = np.random.randint(-100, 100, size=(n_packages, 2))

    asprob = airsimproblem(max_dist, initial, goal, packages_locations)
    now = time.time()
    start = time.time_ns()
    goal_node = search.astar_search(asprob)
    t = time.time_ns() - start
    print(f'done in {time.time() - now:.2f}s\n')

    solution_path = goal_node.path()
    print(f"goal:\n {solution_path}")
    plt.scatter(packages_locations[:, 0], packages_locations[:, 1])
    colors = cm.rainbow(np.linspace(0, 1, n_drones))
    last_node = None
    total_dist = 0
    for node in solution_path:
        if last_node is None:
            last_node = node
            continue
        for drone in range(1, n_drones + 1):
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
    # plt.show()
    # with open(f'solutions/astar/{n_drones}_{n_packages}_{total_dist:.0f}_{t}.pkl', 'wb') as f:
    #     pickle.dump(root, f)
    plt.savefig(f'graphs/astar/{n_drones}_{n_packages}_{total_dist:.0f}_{t}.png')
    plt.close()
    # with open('solution.pkl', 'wb') as f:
    #     pickle.dump(solution_path, f)

    # asrunpath = airsumrunpath(num_drones, packages_locations)
    # asrunpath.follow_path(solution_path)


def mcts(n_drones, n_packages, n_simulations, package_locations):
    initial = hashabledict()
    initial['drones'] = hashabledict({i + 1: 0 for i in range(n_drones)})
    initial['packages'] = hashabledict({i + 1: 0 for i in range(n_packages)})

    goal = hashabledict()
    goal['drones'] = hashabledict({i + 1: 0 for i in range(n_drones)})
    goal['packages'] = hashabledict({i + 1: -1 for i in range(n_packages)})

    root = MonteCarloTreeSearchNode(state=initial, goal=goal, n_simulations=n_simulations,
                                    package_locations=package_locations)
    path = []
    plt.scatter(package_locations[:, 0], package_locations[:, 1])
    colors = cm.rainbow(np.linspace(0, 1, n_drones))
    selected_action = root
    start = time.time_ns()
    while True:
        if selected_action.parent_action != None:
            for drone, destination in selected_action.parent_action.items():
                pos = np.array([0, 0])
                last_pos = np.array([0, 0])
                if destination > 0:
                    pos = package_locations[destination - 1]
                if selected_action.parent.state['drones'][drone] > 0:
                    last_pos = package_locations[selected_action.parent.state['drones'][drone] - 1]
                # if drone == 1:
                #     print(last_pos, pos)
                plt.plot([last_pos[0], pos[0]], [last_pos[1], pos[1]], color=colors[drone - 1])
        path.append(selected_action.state['drones'])
        if selected_action.is_terminal_node():
            break
        selected_action = selected_action.best_action()
    # plt.show()
    t = time.time_ns() - start
    with open(f'solutions/mcts/{n_drones}_{n_packages}_{n_simulations}_{root.minimal_distance:.0f}_{t}.pkl', 'wb') as f:
        pickle.dump(root, f)
    plt.savefig(f'graphs/mcts/{n_drones}_{n_packages}_{n_simulations}_{root.minimal_distance:.0f}_{t}.png')
    plt.close()

    print(root.minimal_distance)
    path.append(selected_action.state['drones'])
    print(path)
    # MonteCarloTreeSearchNode.graph.visualize()
    root.graphviz()
    root.graph.render()


if __name__ == "__main__":
    # for n_drones in (2, 3, 4, 5):
    #     for n_packages in (10, 13, 15, 17, 20):
    #         package_locations = np.random.randint(-100, 100, size=(n_packages, 2))
    #         for n_simulations in (10, 20, 30, 40, 50, 60, 70, 80, 90, 100):
    #             mcts(n_drones, n_packages, n_simulations, package_locations)
    for n_drones in (2, 3, 4, 5):
        for n_packages in (10, 13, 15, 17, 20):
            main(n_drones, n_packages)
    # with open('5_13_10_1679_308176800.pkl', 'rb') as f:
    #     problem = pickle.load(f)
    # path = []
    # selected_action = problem
    # while True:
    #     if selected_action.is_terminal_node():
    #         break
    #     selected_action = selected_action.best_action()
    #     path.append(selected_action.state['drones'])
    #
    # runpath = airsumrunpath(5, problem.package_locations)
    # runpath.follow_path_mcts(path)
