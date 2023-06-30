import airsim
import math
import numpy as np


def getClosestPoint(points):
    closest = [0, 0, 0]
    closestDistance = 1000000
    for point in points:
        distance = math.sqrt(point[0]*point[0] + point[1]*point[1] + point[2]*point[2])
        if (distance < closestDistance):
            closest = [point[0], point[1], point[2]]
            closestDistance = distance
    return closest

def getfurthestPoint(points):
    furthest = [0, 0, 0]
    furthestDistance = 0
    
    for point in points:
        distance = math.sqrt(point[0]*point[0] + point[1]*point[1] + point[2]*point[2])
        if (distance > furthestDistance):
            furthest = [point[0], point[1], point[2]]
            furthestDistance = distance
    
    return furthest

def get_points():

    lidar_data = client.getLidarData()
    
    if (len(lidar_data.point_cloud) < 3):
        return None

    points = np.array(lidar_data.point_cloud, dtype=np.dtype('f4'))
    
    points = np.reshape(points, (int(points.shape[0]/3), 3))
    
    return points.tolist()

client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)
state=client.getMultirotorState()
client.takeoffAsync().join()
points=get_points()

if points is not None:
    
    client.moveToPositionAsync(40,0.060,-0.432,1)
    
    while (True):
        k=client.getGpsData().gnss.geo_point
        points = get_points()
        x, y, z = getfurthestPoint(points)
        distance = math.sqrt(x*x + y*y + z*z)
        
        # if (distance > 5):
        #     client.moveByVelocityAsync(0,0,0,1).join()
        #     break
        #else:
        client.moveByVelocityAsync(x,y,0,1)

else :
    client.moveToPositionAsync(40,0.060,-0.432,1)