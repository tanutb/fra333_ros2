#!/usr/bin/python3
import numpy as np
import rclpy
import sys, os, yaml
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from std_msgs.msg import Float64MultiArray,Float64
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

# from example_interfaces.srv import enableTracker

class tracker(Node):
    
    def __init__(self):
        super().__init__('tracker_node')

        self.q_sub = self.create_subscription(JointState,"/joint_states" ,self.q_callback, 10)
        self.q_ref_sub = self.create_subscription(JointState,"/reference/joint_states" ,self.q_ref_callback, 10)
        self.pub = self.create_publisher(Float64MultiArray,"/joint_velocity_controller/commands",10)

        #Modify This service
        # self.enableTracker_service = self.create_service(enableTracker,'/enableTracker',self.enableTracker_set)
        # self.enableTracker = False
        self.timer = self.create_timer(1/10,self.timer_callback)

        # modify this dummy
        self.q = np.array([0.,0.,0.])

        self.q_ref = np.array([0.,0.,0.])
        self.q_ref_dot = np.array([0.,0.,0.])

        # Change to read data from yaml
        self.Kp = 1.0
        self.Ki = 0.01
        
        self.sum_error_q = 0

    def set_joints_callback(self,req,res):
        self.enableTracker = req.data

    def q_callback(self,msg:JointState):
        self.q = np.array(msg.position)
    
    def q_ref_callback(self,msg:JointState):
        self.q_ref = np.array(msg.position)
        self.q_ref_dot = np.array(msg.velocity)

    
    def timer_callback(self):
        # if(self.enableTracker):
        self.sum_error_q += (self.q_ref - self.q)
        velo_command = list(self.q_ref_dot + (self.Kp * (self.q_ref - self.q)) + self.Ki * self.sum_error_q)
        command = Float64MultiArray()
        command.data = velo_command
        self.pub.publish(command)

def main(args=None):
    rclpy.init(args=args)
    Tracker = tracker()
    try:
        while rclpy.ok():
            rclpy.spin_once(Tracker)
    except KeyboardInterrupt:
        print('repeater stopped cleanly')
    except BaseException:
        print('exception in repeater:', file=sys.stderr)
        raise
    finally:
        Tracker.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()