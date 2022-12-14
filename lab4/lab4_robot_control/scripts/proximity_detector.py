#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool , Float64MultiArray
from lab4_robot_control.module import *
import numpy as np

class proximity_detector(Node):
    def __init__(self):
        super().__init__("proximity_detector")
        self.q_sub = self.create_subscription(JointState,"/joint_states" ,self.q_callback, 10)
        self.p_f_sub = self.create_subscription(Float64MultiArray,"/HI" ,self.p_f_sub_callback, 10)
        self.hasReached_pub = self.create_publisher(Bool,"/hasReached", 10)
        self.timer = self.create_timer(1/10,self.timer_callback)
        self.hasReached = False
        self.p_f = np.array([0.,0.,0.])
        self.q = np.array([0.,0.,0.])
    
    def p_f_sub_callback(self,msg):
        self.p_f = msg.data
    def q_callback(self,msg:JointState):
        self.q = msg.position
        
    def timer_callback(self):
        P = FPK(self.q)
        pos = np.array(P[-1][:3,3])
        print(pos)
        if((abs(np.array(self.p_f) - pos) <= 0.001).all()):
            self.hasReached = True
        else:
            self.hasReached = False
        msg = Bool()
        msg.data = self.hasReached
        self.hasReached_pub.publish(msg)
        print(self.hasReached)
        
        

def main(args=None):
    rclpy.init(args=args)
    node = proximity_detector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()