import numpy as np
from search import Problem
from collections import defaultdict
from itertools import permutations
from utils import hashabledict
from sklearn.cluster import KMeans
import time
import random

random.seed(0)


class airsimproblem(Problem):
    def __init__(self, max_dist, initial, goal, packages_locations):
        super(airsimproblem, self).__init__(initial, goal)
        self.num_drones = len(initial['drones'])
        self.max_dist = max_dist
        self.packages_locations = packages_locations
        kmeans = KMeans(n_clusters=5)
        clusters = kmeans.fit_predict(packages_locations)
        self.package_groups = {package: clusters[package - 1] for package in initial['packages']}
        self.initial = initial
        self.start = time.time()

    def actions(self, state):
        """
        :param state: drones locations & package locations: -1 for destination, positive integers are drones, 0 destination
        1. for each drone -> if at home & have room for package: check packages -> if reachable: lift go to dest, else: stay.
        2. for each drone -> if at destination & have package: go to next destination, else: go home.
        """
        drone_package_pairs = defaultdict(list)
        free_packages = []
        available_drones = []
        unavailable_drones = []
        home = []
        for package, drone in state['packages'].items():
            if drone > 0:
                drone_package_pairs[drone].append(package)
            elif drone == 0:
                free_packages.append(package)

        for drone, location in state['drones'].items():
            if location == 0 and len(drone_package_pairs[drone]) < min(2, len(free_packages)):
                available_drones.append(drone)
            elif location != 0 and len(drone_package_pairs[drone]) == 0:
                home.append(drone)
            else:
                unavailable_drones.append(drone)
        actions = []
        for perm in permutations(free_packages, min(len(available_drones), len(free_packages))):
            action = hashabledict({drone: 0 for drone in state['drones']})
            for i, drone in enumerate(available_drones):
                if i >= len(free_packages):
                    break
                if len(drone_package_pairs[drone]) > 0 and \
                        self.package_groups[drone_package_pairs[drone][0]] != self.package_groups[perm[i]]:
                    action[drone] = drone_package_pairs[drone][0]
                else:
                    action[drone] = -perm[i]  # -x    Pickup package x
            for drone in unavailable_drones:
                if drone_package_pairs[drone]:
                    action[drone] = drone_package_pairs[drone][0]  # x     Go to location of package x
            for drone in home:
                action[drone] = 0  # 0     Go to home
            actions.append(action)
        actions = list(set(actions))
        # print(len(actions))
        if state == self.initial:
            actions = actions[:20]
            # return random.choices(actions, k=20)
        return actions

    def result(self, state, action):
        # print(action)
        # print([location for location in state['packages'].values()])
        new_state = hashabledict()
        new_state['drones'] = hashabledict({drone: location for drone, location in state['drones'].items()})
        new_state['packages'] = hashabledict({package: location for package, location in state['packages'].items()})
        for drone, operation in action.items():
            package = abs(operation)
            if operation == 0:
                new_state['drones'][drone] = 0
            elif operation > 0:
                new_state['drones'][drone] = operation
                new_state['packages'][package] = -1
            else:
                new_state['packages'][package] = drone
        # print(new_state)
        return new_state

    def h(self, node):
        dist = [self.packages_locations[loc - 1] - self.packages_locations[package - 1]
                for package, loc in node.state['packages'].items() if loc != -1]
        cost = np.sqrt(np.sum(np.square(dist))) - node.depth
        cost *= time.time() - self.start
        return cost
