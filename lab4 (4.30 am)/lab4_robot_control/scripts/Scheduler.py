#!/usr/bin/python3
import numpy as np
import rclpy
import sys, os, yaml
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from std_msgs.msg import Float64MultiArray,Float64,Bool
from sensor_msgs.msg import JointState
import yaml
from lab4_robot_interfaces.srv import Enabletracker
import time
class Scheduler(Node):
    def __init__(self):
        super().__init__('Scheduler')
        #Pub to trajectory
        self.Pi_pub = self.create_publisher(Float64MultiArray,"/Pi" , 10)
        self.Pf_pub = self.create_publisher(Float64MultiArray,"/Pf" , 10)
        self.T_pub = self.create_publisher(Float64,"/T" , 10)
        
        #Subfrom Proximity
        self.hasReached_sub = self.create_subscription(Bool,"/hasReached" ,self.hasReached_callback,10)
        # #service EnableTracker
        self.enableTracker_Cli = self.create_client(Enabletracker, '/enableTracker')
        self.enableTracker_req = Enabletracker.Request()

        self.enableTracker_pub = self.create_publisher(Bool, '/enableTracker',10)

        self.Hi = self.create_publisher(Float64MultiArray, '/HI',10)
        #modify this
        self.hasReached = False
        self.K = 1
        self.current_p = [0.31,0.,0.135]
        self.next_p = [0.,0.,0.]
        self.T = 4
        # x - 0.15  y - 0.05 z = 0.134999
        
        with open(r'lab4_robot_control/config/via_point.yaml') as file:
            via_point = yaml.load(file, Loader=yaml.FullLoader)
        self.via_points = via_point['via_point']
        self.N = len(self.via_points)

        if(self.K<self.N):
            self.enableTracker_req.tracker_enable.data = True
            self.enableTracker_Cli.call_async(self.enableTracker_req)
        else:
            self.enableTracker_req.tracker_enable.data = False
            self.enableTracker_Cli.call_async(self.enableTracker_req)
        self.S()


    def hasReached_callback(self,msg):
        #Pub P_init and P_final to trajectory generator
        if(msg.data == True):
            self.S()
            
    def S(self):
        if(self.K<self.N):
            pub_msg = Float64MultiArray()
            # self.current_p = self.via_points['via_points'][self.K-1]
            # self.next_p = self.via_points['via_points'][self.K]
            self.current_p = self.via_points[self.K-1]
            self.next_p = self.via_points[self.K]           
            pub_msg.data = list(self.current_p)
            self.Pi_pub.publish(pub_msg)
            
            pub_msg.data = list(self.next_p)
            self.Pf_pub.publish(pub_msg)
            
            pub_msg = Float64()
            pub_msg.data = float(self.T)
            self.T_pub.publish(pub_msg)
            
            a = Bool()
            a.data = True
            print(self.K)
            pub_msg = Float64MultiArray()
            pub_msg.data = list(self.next_p)
            self.Hi.publish(pub_msg)
            self.enableTracker_pub.publish(a)
            self.K +=1
        elif(self.K>=self.N and self.enableTracker_req.tracker_enable.data == True):
            self.enableTracker_req.tracker_enable.data = False
            self.enableTracker_Cli.call_async(self.enableTracker_req)

            
            

def main(args=None):
    rclpy.init(args=args)
    scheduler = Scheduler()
    try:
        while rclpy.ok():
            rclpy.spin_once(scheduler)
    except KeyboardInterrupt:
        print('repeater stopped cleanly')
    except BaseException:
        print('exception in repeater:', file=sys.stderr)
        raise
    finally:
        scheduler.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()












#f
# self.via_points  = [[0.15,0.15,0.1349999],[0.2,0.15,0.1349999],[0.2,0.15,0.1349999],[0.2,0.1,0.1349999],[0.2,0.1,0.1349999],[0.2,0.15,0.1349999],[0.2,0.15,0.1349999],[0.25,0.15,0.1349999],[0.25,0.15,0.1349999],[0.25,0.1,0.1349999]]
# #i
# self.via_points = [[0.25,0.05,0.1349999],[0.2,0.05,0.1349999],[0.2,0.05,0.1349999],[0.15,0.05,0.1349999],[0.15,0.05,0.1349999],[0.25,0.05,0.1349999]]
# #b
# self.via_points = [[0.25,0.,0.1349999],[0.225,-0.05,0.1349999],[0.225,-0.05,0.1349999],[0.2,0.,0.1349999],[0.2,0.,0.1349999],[0.175,-0.05,0.1349999],[0.175,-0.05,0.1349999],[0.15,0.,0.1349999],[0.15,0.,0.1349999],[0.25,0.,0.1349999]]
# #o
# self.via_points = [[0.25,-0.15,0.1349999],[0.2,-0.2,0.1349999],[0.2,-0.2,0.1349999],[0.15,-0.15,0.1349999],[0.15,-0.15,0.1349999],[0.2,-0.1,0.1349999],[0.2,-0.1,0.1349999],[0.25,-0.15,0.1349999]]