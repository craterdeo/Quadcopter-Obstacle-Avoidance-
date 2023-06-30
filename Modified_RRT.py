import airsim
from lidar import lidar
import numpy as np
import random
from math import ceil
from datetime import datetime
import csv
import pandas as pd
from datetime import datetime

class drone:

    def __init__(self):

        self.client = airsim.MultirotorClient()
        self.l=lidar()
        self.max_dist=2
        self.current_pose=None
        self.final=[40,0,12,-0.432] #-0.432 #12
        self.buffer=0.2
        self.vel=1
        self.flight_info=[]
        self.default_vel=1

    def round(self,a):
        return 0.5 * ceil(2.0 * a)

    def startup(self):
        self.client.confirmConnection()
        self.client.enableApiControl(True)
        self.client.armDisarm(True)
        self.client.takeoffAsync().join()
        self.current_pose=self.get_current_pose()

    def next_pose(self):
        
        lidar_points=self.l.points()

        if lidar_points is not None:
            cost={}
            collission_heuristic={}
            paths=[]
            
        ##################### generating all possible nodes ######################################
            
            for x in range(-self.max_dist,self.max_dist+1):
                for y in range(-self.max_dist,self.max_dist+1):
                    paths.append([x,y,0])
            paths.remove([0,0,0])
            
        ##################### determining safe nodes ##############################################
            
            for point in lidar_points:  
                
                if [ceil(point[0]),ceil(point[1]),0] in paths:
                    paths.remove([ceil(point[0]),ceil(point[1]),0])
        
        ###################### edge case ##########################################################

            if paths==[]:
                return [self.final[0]-self.current_pose[0],self.final[1]-self.current_pose[1],self.final[2]-self.current_pose[2]]
        
        ####################### calculate heuristic distance of node to object #####################
            
            for s_point in paths:
                colission=1000
                for o_point in lidar_points:
                    
                    col=abs(o_point[0]-s_point[0])+abs(o_point[1]-s_point[1])
                    
                    if col<colission:
                        colission=col
                
                collission_heuristic[str(s_point)]=1000-colission

        ####################### calculate heuristic distance of node to goal ########################

            for point in paths:
                distance=abs(self.final[0]-point[0])+abs(self.final[1]-point[1])
                #cost[distance+(collission_heuristic[str(point)])*1.2]=point
                cost[distance+(collission_heuristic[str(point)])*1.5]=point  

        ####################### returning the ideal node ############################################
            
            net_heuristics=list(cost.keys())
            net_heuristics.sort()  

        ######################## logging data #######################################################
            
            self.write_data([cost[net_heuristics[0]],str(1000-collission_heuristic[str(cost[net_heuristics[0]])])])
            
        ##############################################################################################
            
            # self.vel=(1000-collission_heuristic[str(cost[net_heuristics[0]])])*self.default_vel
            return cost[net_heuristics[0]]

        
        else :
            # self.vel=self.default_vel
            return [self.final[0]-self.current_pose[0],self.final[1]-self.current_pose[1],self.final[2]-self.current_pose[2]] 

    def is_safe(self,loc,points):

        for point in points:

            current=self.get_pos()
            
            if (self.round(point[0])==loc[0] and self.round(point[1])==loc[1]):
                return False
    
        return True

    def get_current_pose(self):
        return [self.client.simGetVehiclePose().position.x_val,self.client.simGetVehiclePose().position.y_val,self.client.simGetVehiclePose().position.z_val]
    
    def get_future_pose(self,offset):

        self.current_pose=self.get_current_pose()
        newx=self.current_pose[0]+offset[0]
        newy=self.current_pose[1]+offset[1]
        newz=-0.432

        return [newx,newy,newz]

    def start(self):
        
        self.startup()
        
        while True:

            #self.write(self.current_pose)
            
            offset=self.next_pose()
            
            newx,newy,newz=self.get_future_pose(offset) 

            print("goin towards point : ",offset)
            
            self.client.moveToPositionAsync(newx,newy,newz,self.vel)

    def test(self):
        self.startup()
        self.client.moveToPositionAsync(20,12,-0.432,1)
        current_pose=self.client.simGetVehiclePose()
        print(current_pose)
    
    def write_data(self,data):
        
        data.append(datetime.now().strftime("%H:%M:%S"))

        with open("path_data2.csv", 'a') as csvfile:
            
            spamwriter = csv.writer(csvfile, delimiter=',')
            
            spamwriter.writerow(data)


if __name__== "__main__" :
    
    d=drone()
    d.start()
    print("reached!!!!!")