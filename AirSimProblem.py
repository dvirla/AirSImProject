import numpy as np
from search import Problem
from collections import defaultdict
from itertools import permutations


class airsimproblem(Problem):
    def __init__(self, num_drones, max_dist, initial, goal, packages_locations):
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
        legal_actions = {}
        drone_package_pairs = defaultdict(list)
        free_packages = []
        avilable_drones = []
        for package, drone in enumerate(state['packages']):
            if drone > 0:
                drone_package_pairs[drone].append(package)
            elif drone == 0:
                free_packages.append(package)

        for drone, location in enumerate(state['drones']):
            if location == 0 and len(drone_package_pairs[drone]) < 2:
                avilable_drones.append(drone)
            elif len(drone_package_pairs[drone]) == 2:
                legal_actions[f'drone_{drone}'] = -1


    # def TakeOff(self):
    #     for drone in range(self.num_drones):
    #         self.client.takeoffAsync(vehicle_name=f'drone_{drone}').join()
    #
    #
    # def Land(self):
    #     for drone in range(self.num_drones):
    #         self.client.landAsync(vehicle_name=f'drone_{drone}').join()


