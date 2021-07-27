import numpy as np
from search import Problem
from collections import defaultdict
from itertools import permutations
from utils import hashabledict
import random


class airsimproblem(Problem):
    def __init__(self, max_dist, initial, goal, packages_locations):
        super(airsimproblem, self).__init__(initial, goal)
        self.num_drones = len(initial['drones'])
        self.max_dist = max_dist
        self.packages_locations = packages_locations
        # self.client = airsim.MultirotorClient()
        # self.client.reset()
        # for drone in range(num_drones):
        #     self.client.enableApiControl(True, vehicle_name=f'drone_{drone}')
        #     self.client.armDisarm(True, vehicle_name=f'drone_{drone}')
        #     self.client.takeoffAsync(vehicle_name=f'drone_{drone}').join()
        #     self.client.moveToPositionAsync(0, 0, -200, 10, vehicle_name=f'drone_{drone}').join()

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
        for package, drone in enumerate(state['packages']):
            if drone > 0:
                drone_package_pairs[drone].append(package)
            elif drone == 0:
                free_packages.append(package)

        for drone, location in enumerate(state['drones']):
            if location == 0 and len(drone_package_pairs[drone]) < 2:
                available_drones.append(drone)
            elif location != 0 and len(drone_package_pairs[drone]) == 0:
                home.append(drone)
            else:
                unavailable_drones.append(drone)
        actions = []
        for perm in permutations(free_packages, min(len(available_drones), len(free_packages))):
            action = [0] * len(state['drones'])
            for i, drone in enumerate(available_drones):
                if i >= len(free_packages):
                    break
                action[drone] = -perm[i]  # -x    Pickup package x
            for drone in unavailable_drones:
                action[drone] = drone_package_pairs[drone][0]  # x     Go to location of package x
            for drone in home:
                action[drone] = 0  # 0     Go to home
            actions.append(tuple(action))
        print(actions)
        if len(actions) > 20:
            return random.sample(actions, 20)
        if len(actions) == 0:
            x = 5
        return actions

    def result(self, state, action):
        print(action)
        print([location for location in state['packages']])
        new_state = hashabledict()
        new_state['drones'] = list(state['drones'])
        new_state['packages'] = list(state['packages'])
        for drone, operation in enumerate(action):
            if operation >= 0:
                new_state['drones'][drone] = operation
                new_state['packages'][operation] = -1
            else:
                new_state['packages'][operation] = drone
        new_state['drones'] = tuple(new_state['drones'])
        new_state['packages'] = tuple(new_state['packages'])
        print(new_state)
        return new_state

    def h(self, node):
        state = node.state
        return -100 * len([0 for location in state['packages'] if location == -1])

    # def TakeOff(self):
    #     for drone in range(self.num_drones):
    #         self.client.takeoffAsync(vehicle_name=f'drone_{drone}').join()
    #
    #
    # def Land(self):
    #     for drone in range(self.num_drones):
    #         self.client.landAsync(vehicle_name=f'drone_{drone}').join()
