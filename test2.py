import airsim
import numpy as np
from lidar import lidar
import math
import csv

vel=2

client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)
state=client.getMultirotorState()
client.takeoffAsync().join()

#start : -1.2200,0.060,-0.432
#end : 2.860,0.060,-0.432


# Set up obstacle avoidance algorithm
prev_dir=0
dircount=0

def avoid_obstacles():
    lidar_data = client.getLidarData()
    if (len(lidar_data.point_cloud) < 3):
        return None

    points = np.array(lidar_data.point_cloud, dtype=np.dtype('f4'))
    
    points = np.reshape(points, (int(points.shape[0]/3), 3))

    obstacles = []
    for point in points:
        
        if point[2] < 0:
            continue
        
        if np.linalg.norm(point) < 1.5:
            obstacles.append(point)

    if len(obstacles) > 0:
        centroid = np.mean(obstacles, axis=0)
        return centroid
    
    else:
        return None

def manuver(avoid_vector):
    thresh=2
    
    if (avoid_vector[0]<thresh or avoid_vector[1]<thresh and avoid_vector[1]>0):
        
       
        while(avoid_vector[0]<thresh and avoid_vector[1]<thresh):
            
            velocity_command = [avoid_vector[0],-avoid_vector[1], 0]
            velocity_command /= np.linalg.norm(velocity_command)
            velocity_command*=vel
            velx=0
            vely=0

            print(avoid_vector)

            if(abs(avoid_vector[0])<abs(avoid_vector[1])):
                
                velx=velocity_command[0]
                vely=0                  
                print("obstacle close in y")

            else:
                velx=0
                vely=velocity_command[1]
                print("obstacle close in x")

            client.moveByVelocityAsync(
                0,
                0,
                0,
                1
            ).join()


            client.moveByVelocityAsync(
                velx,
                vely,
                0,
                1
            ).join()

            avoid_vector = avoid_obstacles()
            if(avoid_vector is None):
                break

    else:
        print("possible collission")
        client.moveToPositionAsync(20,0.060,-0.432,vel)
        #client.moveByVelocityAsync(avoid_vector[1]*vel,0,0,1).join()

def write():

    lidar_data=client.getLidarData()

    if (len(lidar_data.point_cloud) < 3):
        return None
    
    points = np.array(lidar_data.point_cloud)
    
    points = np.reshape(points, (int(points.shape[0]/3), 3))
    points=list(points)

    with open("data.csv", 'w') as csvfile:

        csvwriter = csv.writer(csvfile)
        csvwriter.writerows(points)

def get_gps():
    
    k=client.getGpsData().gnss.geo_point
    lat=k.latitude
    lon=k.longitude
    alt=k.altitude

    #trig variables
    cosLat = math.cos(lat)
    sinLat = math.sin(lat)
    cosLon = math.cos(lon)
    sinLon = math.sin(lon)

    #Constants
    rad = 6378137 + alt
    f = 1.0 / 298.257224
    C = 1.0 / math.sqrt(cosLat * cosLat + (1-f) * (1-f) * sinLat * sinLat)
    S = (1.0 - f) * (1.0 - f) * C
    h = 0.0

    #return values
    value = []
    value.append((rad * C + h) * cosLat * cosLon)
    value.append((rad * C + h) * cosLat * sinLon)
    value.append((rad * S + h) * sinLat)

    return value

def start():
    l=lidar()
    while client.ping():
        # Get obstacle avoidance vector
        avoid =l.get_nearset_vector()

        for distance in avoid.keys():

            avoid_vector=avoid[distance]
        
            if avoid_vector is not None and distance<2:
                
                velx=0
                vely=0
                x=avoid_vector[1]
                y=avoid_vector[0]
                # Convert avoid vector to velocity command
                velocity_command = [avoid_vector[1], -avoid_vector[0], 0]
                velocity_command /= np.linalg.norm(velocity_command)
                velocity_command *= vel  # Set desired velocity
                
                print("distance to object : ",distance)
                #pos=get_gps()

                if abs(x)<abs(y):
                    print("collission in x !!!")
                    # client.moveToPositionAsync(40,0.060,-0.432,vel)
                    client.moveByVelocityAsync(0,0,0,1).join()
                    client.moveByVelocityAsync(0,velocity_command[1],0,1).join()

                else :
                    print("collission in y !!!")
                    client.moveByVelocityAsync(0,0,0,1).join()
                    client.moveByVelocityAsync(velocity_command[0],0,0,1).join()
            

            # Otherwise, continue flying forward
            else:
                print("no collission ....")
                #client.moveToPositionAsync(40,0.060,-0.432,vel)
                client.moveByVelocityAsync(
                    vel,  # Set desired velocity
                    0,
                    0,
                    1
                ).join()

def old():

    while client.ping():
        # Get obstacle avoidance vector
            l=lidar()
            avoid_vector=avoid_obstacles()
            pos=get_gps()

            if avoid_vector is not None :

                distance=abs(avoid_vector[0])+abs(avoid_vector[1])+abs(avoid_vector[2])
                if (distance<5):

                    velx=0
                    vely=0
                    x=avoid_vector[1]
                    y=avoid_vector[0]
                    # Convert avoid vector to velocity command
                    velocity_command = [avoid_vector[1], -avoid_vector[0], 0]
                    velocity_command /= np.linalg.norm(velocity_command)
                    velocity_command *= vel  # Set desired velocity
                    
                    # print("distance to object : ",distance)
                    #pos=get_gps()

                    if abs(x)<abs(y):

                        print("collission in x !!!")
                        # client.moveToPositionAsync(40,0.060,-0.432,vel)
                        client.moveByVelocityAsync(0,0,0,1).join()
                        client.moveByVelocityAsync(0,-np.sign(y)*vel,0,1)

                    else :
                        print("collission in y !!!")
                        client.moveByVelocityAsync(0,0,0,1).join()
                        client.moveByVelocityAsync(vel,0,0,1).join()
                else :
                        client.moveByVelocityAsync(vel*avoid_vector[1],0,0,1)


            # Otherwise, continue flying forward
            else:
                print("no collission ....")
                client.moveToPositionAsync(40,0.060,-0.432,vel)

client.moveToPositionAsync(40,0.060,-0.432,vel)
old()