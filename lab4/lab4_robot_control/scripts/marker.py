#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool , Float64MultiArray
from visualization_msgs.msg import MarkerArray , Marker
from geometry_msgs.msg import Point
from lab4_robot_control.module import *
import numpy as np

class marker_node(Node):
    def __init__(self):
        super().__init__("asdads")
        self.Marker_pub = self.create_publisher(Marker,"/visualization_marker",10)
        self.q_sub = self.create_subscription(JointState,"/joint_states" ,self.q_callback, 10)
        self.Marker = Marker()
        self.Marker.header.frame_id = "/world"
        self.Marker.type = 8
        # self.Marker.type = 4      
        self.Marker.scale.x = 0.02
        self.Marker.scale.y = 0.02
        self.Marker.scale.z = 0.02
        
        self.Marker.color.r = 255/255 
        self.Marker.color.g = 165/255
        self.Marker.color.b = 0.0
        self.Marker.color.a = 1.0

        self.timer = self.create_timer(1/10,self.timer_callback)
        
        self.position = np.array([0.,0.,0.])
        
    def q_callback(self,msg):
        temp = FPK(msg.position)
        self.position = temp[-1][:3,3]    
           
    def timer_callback(self):
        
        now = self.get_clock().now()
        self.Marker.header.stamp = now.to_msg()
        pos = Point()
        pos.x = self.position[0]
        pos.y = self.position[1]
        pos.z = self.position[2]
        if(pos.z < 0.145):
            self.Marker.points.append(pos)
            self.Marker_pub.publish(self.Marker)
    
        

def main(args=None):
    rclpy.init(args=args)
    node = marker_node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()