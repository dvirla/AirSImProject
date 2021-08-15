import time

import airsim
import threading
from time import sleep


class airsumrunpath():
    def __init__(self, num_drones, packages_locations):
        self.num_drones = num_drones
        self.packages_locations = packages_locations
        self.client = airsim.MultirotorClient()
        self.client.reset()
        self.speed = 30
        self.height = -100
        for drone in range(1, self.num_drones + 1):
            self.client.enableApiControl(True, vehicle_name=f'drone_{drone}')
            self.client.armDisarm(True, vehicle_name=f'drone_{drone}')
            self.client.takeoffAsync(vehicle_name=f'drone_{drone}')
            # self.client.moveToPositionAsync(0, 0, self.height, self.speed, vehicle_name=f'drone_{drone}')

    def reset(self):
        for drone in range(1, self.num_drones + 1):
            self.client.goHomeAsync(vehicle_name=f"drone_{drone}").join()

    def run_drone_path(self, drone, path):
        self.client.enableApiControl(True, vehicle_name=f'drone_{drone}')
        self.client.armDisarm(True, vehicle_name=f'drone_{drone}')
        print(f'drone {drone} takeoff')
        self.client.takeoffAsync(vehicle_name=f'drone_{drone}')
        print(f'drone {drone} height {self.height}')
        self.client.moveToPositionAsync(0, 0, self.height, self.speed, vehicle_name=f'drone_{drone}')
        for action in path:
            print(f'drone {drone} action {action}')
            # if action == 'home':
                # self.clients[drone].goHomeAsync(vehicle_name=f"drone_{drone}").join()
            # else:
            self.client.moveToPositionAsync(*action)
            sleep(1)

    def follow_path_astar(self, path):
        drone_paths = {i: [] for i in range(1, self.num_drones + 1)}
        for node in path:
            drones_destinations = node.state['drones']
            for drone in range(1, self.num_drones + 1):
                if drones_destinations[drone] == 0:
                    # drone_paths[drone].append('home')
                    # self.client.goHomeAsync(vehicle_name=f"drone_{drone}")
                    drone_paths[drone].append(airsim.Vector3r(0, 0, self.height))
                    drone_paths[drone].append(airsim.Vector3r(0, 0, self.height - 20))
                    drone_paths[drone].append(airsim.Vector3r(0, 0, self.height))
                else:
                    x_val, y_val = self.packages_locations[drones_destinations[drone] - 1]
                    drone_paths[drone].append(airsim.Vector3r(int(x_val), int(y_val), self.height))
                    drone_paths[drone].append(airsim.Vector3r(int(x_val), int(y_val), self.height - 30))
                    drone_paths[drone].append(airsim.Vector3r(int(x_val), int(y_val), self.height))
                    # self.client.moveToPositionAsync(int(x_val), int(y_val), self.height, self.speed)
            # sleep(0.5)
        for drone, drone_path in drone_paths.items():
            self.client.moveOnPathAsync(drone_path, self.speed, vehicle_name=f'drone_{drone}')
            time.sleep(0.1)

    def follow_path_mcts(self, path):
        drone_paths = {i: [] for i in range(1, self.num_drones + 1)}
        for drones_destinations in path:
            for drone in range(1, self.num_drones + 1):
                if drones_destinations[drone] == 0:
                    # drone_paths[drone].append('home')
                    # self.client.goHomeAsync(vehicle_name=f"drone_{drone}")
                    drone_paths[drone].append(airsim.Vector3r(0, 0, self.height))
                    drone_paths[drone].append(airsim.Vector3r(0, 0, 0))
                    drone_paths[drone].append(airsim.Vector3r(0, 0, self.height))
                else:
                    x_val, y_val = self.packages_locations[drones_destinations[drone] - 1]
                    drone_paths[drone].append(airsim.Vector3r(int(x_val), int(y_val), self.height))
                    drone_paths[drone].append(airsim.Vector3r(int(x_val), int(y_val), 0))
                    drone_paths[drone].append(airsim.Vector3r(int(x_val), int(y_val), self.height))
                    # self.client.moveToPositionAsync(int(x_val), int(y_val), self.height, self.speed)
            # sleep(0.5)
        for drone, drone_path in drone_paths.items():
            self.client.moveOnPathAsync(drone_path, self.speed, vehicle_name=f'drone_{drone}')
            time.sleep(0.1)

