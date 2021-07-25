# ready to run example: PythonClient/multirotor/hello_drone.py
import airsim
import numpy as np
import os

# connect to the AirSim simulator
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)

# Async methods returns Future. Call join() to wait for task to complete.
client.takeoffAsync().join()
client.moveToPositionAsync(-20, 17, -50, 5).join()
client.landAsync().join()
client.getHomeGeoPoint() #lat,long,altitude,time
client.getGpsData() #velocity: xyz + lat,long,altitude,time
client.getMultirotorState() # gps data + kinematics + position
client.goHomeAsync()


# add new vehicle
vehicle_name = "Drone2"
pose = airsim.Pose(airsim.Vector3r(0, 0, 0), airsim.to_quaternion(0, 0, 0))

client.simAddVehicle(vehicle_name, "simpleflight", pose)
client.enableApiControl(True, vehicle_name)
client.armDisarm(True, vehicle_name)
client.takeoffAsync(10.0, vehicle_name)

