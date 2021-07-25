import airsim

class dronecontroller:
    def __init__(self, num_drones):
        self.num_drones = num_drones
        self.client = airsim.MultirotorClient()
        for drone in range(num_drones):
            self.client.enableApiControl(True, vehicle_name=f'drone_{drone}')
            self.client.armDisarm(True, vehicle_name=f'drone_{drone}')

    def TakeOff(self):
        for drone in range(self.num_drones):
            self.client.takeoffAsync(vehicle_name=f'drone_{drone}').join()


    def Land(self):
        for drone in range(self.num_drones):
            self.client.landAsync(vehicle_name=f'drone_{drone}').join()


