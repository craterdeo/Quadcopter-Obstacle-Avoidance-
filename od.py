import airsim
import numpy as np
from lidar import lidar
import math
from math import ceil
import csv
import time

vel=2
z=0.432

client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)
state=client.getMultirotorState()
client.takeoffAsync().join()

#start : -1.2200,0.060,-0.432
#end : 2.860,0.060,-0.432

max_distance=1.2

class drone_data:
    
    def __init__(self):
        #40,0.060,-0.432,vel
        self.initial_pose=client.getGpsData().gnss.geo_point
        self.final_pose=client.getGpsData().gnss.geo_point
        self.final_pose.longitude=self.initial_pose.longitude+40
        self.final_pose.latitude=self.initial_pose.latitude+0.060
        self.final_pose.altitude=self.initial_pose.altitude
        self.current_loc=0

    def current_loc(self):
        self.current_loc=client.getGpsData().gnss.geo_point

    # def destination_offset()    

def get_points():

    lidar_data = client.getLidarData()
    
    if (len(lidar_data.point_cloud) < 3):
        return None

    points = np.array(lidar_data.point_cloud, dtype=np.dtype('f4'))
    
    points = np.reshape(points, (int(points.shape[0]/3), 3))
    
    return points

def avoid_obstacles(points):

    obstacles = points_obstacle(points)
    #print(points)

    if obstacles is not None:
        # centroid = np.mean(obstacles, axis=0)
        x=0;y=0;z=0
        i=0
        for point in points:
            if abs(point[0])>x:
                x=i
            else :
                y=i
            z+=point[2]
            z=z/(i+1)
            i+=1
        return [points[x][0],points[y][1],z]
    else:
        return None

def points_obstacle(points):
    
    obstacles_points = [p for p in points if ( p[0]**2 < max_distance**2 ) or (p[1]**2 < max_distance**2)]
    
    return obstacles_points

def write():

    lidar_data=client.getLidarData()

    if (len(lidar_data.point_cloud) < 3):
        return None
    
    points = lidar_data.point_cloud
    
    points = np.reshape(points, (int(len(points)/3), 3))
    points=points.tolist()
    
    for point in points:
        point.append(time.time())

    with open("data.csv", 'w') as csvfile:

        csvwriter = csv.writer(csvfile)
        csvwriter.writerows(points)

def rotate(point):
    q = airsim.to_quaternion(0, 0, np.sign(point[0])*1.57)
    client.simCharSetHeadRotation(q)

def old():

    while client.ping():
        # Get obstacle avoidance vector
            points=get_points()

            if points is not None :
                avoid_vector=avoid_obstacles(points)
                x=avoid_vector[0]
                y=avoid_vector[1]
            
                    
                if abs(x)<max_distance and abs(y)>max_distance:

                    print("collission in x !!!")
                    print(avoid_vector)
                    client.moveByVelocityAsync(0,0,0,1).join()
                    client.moveByVelocityAsync(0,-ceil(y),0,1).join()
                    

                elif abs(y)<max_distance and abs(x)>max_distance :
                    
                    print("collission in y !!!")
                    print(avoid_vector)
                    client.moveByVelocityAsync(0,0,0,1).join()
                    client.moveByVelocityAsync(ceil(x),0,0,1).join()
                
                else :
                    
                    print("moving ....")
                    client.moveToPositionAsync(40,0.060,-0.432,1)


            else :
                    client.moveToPositionAsync(40,0.060,-0.432,vel)
                
            # else:
            #     print("no collission ....")
            #     print(avoid_vector)
            #     if(abs(x)>abs(y)):
            #         client.moveByVelocityAsync(x*vel,0,0,1)
            #     else:
            #         client.moveByVelocityAsync(0,vel*y,0,1)

client.moveToPositionAsync(40,0.060,-0.432,vel)
old()
