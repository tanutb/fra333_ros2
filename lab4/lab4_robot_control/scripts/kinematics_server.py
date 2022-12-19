#!/usr/bin/python3
import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from lab4_robot_control.module import *
import sys


class kinematics_server(Node):
    def __init__(self):
        super().__init__('dummy_node')
        if (sys.argv[1] == "Inverse"):
            print("Inverse")
            self.p_ref_sub = self.create_subscription(JointState,"/p_ref",self.p_ref_sub_callback,10)
            self.q_ref_pub = self.create_publisher(JointState,"/q_ref",10)
            self.p_ref = np.array([0.,0.,0.])
            
            
            
            
            
            
        else:
            print("Forward")
            self.p_dot_ref_sub = self.create_subscription(Float64MultiArray,"/p_dot_ref",self.p_dot_ref_sub_callback,10)
            self.q_dot_ref_pub = self.create_publisher(JointState,"/q_dot_ref",10)
        
        
        self.timer = self.create_timer(1/10,self.timer_callback)
        
        self.p_ref = np.array([])
        self.p_dot_ref = np.array([])
        
        self.q_ref = np.array([])
        self.q_dot_ref = np.array([]) 
        
         
    def p_ref_sub_callback(self,msg):
        self.p_ref = msg.data
    
    def p_dot_ref_sub_callback(self,msg):
        self.p_dot_ref = msg.data   
    
    def timer_callback(self):
        gamma = [1,1]
        self.q_ref = IPK(gamma[0],gamma[1],self.p_ref[0],self.p_ref[1],self.p_ref[2])
        self.q_dot_ref = IVK(self.q_ref,self.p_dot_ref)
        '''
        PUB
        '''
        pass
        
    

def main(args=None):
    rclpy.init(args=args)
    node = kinematics_server()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
