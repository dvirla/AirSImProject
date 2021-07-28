# ready to run example: PythonClient/multirotor/hello_drone.py
import airsim

import search
from AirSimProblem import airsimproblem
from utils import hashabledict
import numpy as np
import time

np.random.seed(0)


num_drones = 5
num_packages = 20
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
print(search.astar_search(asprob))
print(f'done in {time.time() - now:.2f}s')




# # connect to the AirSim simulator
# client = airsim.MultirotorClient()
# client.confirmConnection()
# client.enableApiControl(True)
# client.armDisarm(True)
#
# # Async methods returns Future. Call join() to wait for task to complete.
# client.takeoffAsync().join()
# client.moveToPositionAsync(-20, 17, -50, 5).join()
# client.landAsync().join()
# client.getHomeGeoPoint() #lat,long,altitude,time
# client.getGpsData() #velocity: xyz + lat,long,altitude,time
# client.getMultirotorState() # gps data + kinematics + position
# client.goHomeAsync()
#
#
# # add new vehicle
# vehicle_name = "Drone2"
# pose = airsim.Pose(airsim.Vector3r(0, 0, 0), airsim.to_quaternion(0, 0, 0))
#
# client.simAddVehicle(vehicle_name, "simpleflight", pose)
# client.enableApiControl(True, vehicle_name)
# client.armDisarm(True, vehicle_name)
# client.takeoffAsync(10.0, vehicle_name)

