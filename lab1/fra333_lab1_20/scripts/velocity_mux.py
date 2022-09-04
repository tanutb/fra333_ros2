#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

# import all other neccesary libraries
from std_msgs.msg import Float64
import sys

class VelocityMux(Node):
    def __init__(self):
        # get the rate from argument or default
        super().__init__('velocity_multiplexer')
        if (sys.argv[1]):
            self.rate = float(sys.argv[1])
        else:
            self.rate = 5.0
        # add codes here
        self.angular_subscription = self.create_subscription(Float64,'/angular/noise',self.angular_vel_sub_callback,10)
        self.linear_subscription = self.create_subscription(Float64,'/linear/noise',self.linear_vel_sub_callback,10)
       
        # additional attributes
        self.cmd_vel = Twist()
        self.cmd_pub = self.create_publisher(Twist,'/turtle1/cmd_vel',10)
        self.get_logger().info(f'Starting {self.get_name()}')
        self.timer = self.create_timer(1/self.rate,self.timer_callback)
        
    def linear_vel_sub_callback(self,msg:Float64):
        self.cmd_vel.linear.x = msg.data
    
    def angular_vel_sub_callback(self,msg:Float64):
        self.cmd_vel.angular.z = msg.data
    
    def timer_callback(self):
        self.cmd_pub.publish(self.cmd_vel)

def main(args=None):
    rclpy.init(args=args)
    node = VelocityMux()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
