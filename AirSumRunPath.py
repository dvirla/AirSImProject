import airsim
from time import sleep


class airsumrunpath():
    def __init__(self, num_drones, packages_locations):
        self.num_drones = num_drones
        self.packages_locations = packages_locations
        self.client = airsim.MultirotorClient()
        self.client.reset()
        self.speed = 10
        self.height = -200
        for drone in range(1, self.num_drones + 1):
            self.client.enableApiControl(True, vehicle_name=f'drone_{drone}')
            self.client.armDisarm(True, vehicle_name=f'drone_{drone}')
            self.client.takeoffAsync(vehicle_name=f'drone_{drone}').join()
            self.client.moveToPositionAsync(0, 0, self.height, self.speed, vehicle_name=f'drone_{drone}').join()

    def reset(self):
        for drone in range(1, self.num_drones + 1):
            self.client.goHomeAsync(vehicle_name=f"drone_{drone}").join()

    def follow_path(self, path):
        for node in path:
            drones_destinations = node.state['drones']
            for drone in range(1, self.num_drones + 1):
                if drones_destinations[drone] == 0:
                    self.client.goHomeAsync(vehicle_name=f"drone_{drone}")
                else:
                    x_val, y_val = self.packages_locations[drones_destinations[drone][0]]
                    self.client.moveToPositionAsync(x_val, y_val, self.height, self.speed)
            sleep(0.5)

