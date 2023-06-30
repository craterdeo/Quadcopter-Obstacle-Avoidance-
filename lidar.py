import airsim
import numpy as np

class lidar:

    def __init__(self):

        self.client = airsim.MultirotorClient()

    def get_obstacles(self):

        lidar_data = self.client.getLidarData()

        lidar_segment_set=set(lidar_data.segmentation)
        
        if (len(lidar_data.point_cloud) < 3):
            return None

        points = np.array(lidar_data.point_cloud)
        
        points = np.reshape(points, (int(points.shape[0]/3), 3))

        objects={}

        lidar_segment=list(lidar_segment_set)

        for key in lidar_segment:
            objects[key]=[0,0,0]
        
        lidar_segment=lidar_data.segmentation
        
        for i in range(len(points)):
        
            point=points[i]
            objects[lidar_segment[i]]=[(objects[lidar_segment[i]][0]+point[0])/2,
                                        (objects[lidar_segment[i]][1]+point[1])/2,
                                        (objects[lidar_segment[i]][2]+point[2])/2
                                    ]
        return objects
    
    def distance(self,point):
        return abs(point[0]-0)+abs(point[1]-0)

    def get_nearset_vector(self):
        
        objects=self.get_obstacles()
        
        if objects is None:
            return None
        
        distances={}
        
        for key in objects.keys():
            distances[self.distance(objects[key])]=objects[key]
        
        distances_keys=list(distances.keys())
        distances_keys.sort()
        sorted_dict = {i: distances[i] for i in distances_keys}
        
        return sorted_dict
    
    def points(self):
        
        lidar_data = self.client.getLidarData()

        lidar_segment_set=set(lidar_data.segmentation)
        
        if (len(lidar_data.point_cloud) < 3):
            return None

        points = np.array(lidar_data.point_cloud)
        
        points = np.reshape(points, (int(points.shape[0]/3), 3))

        return points.tolist()