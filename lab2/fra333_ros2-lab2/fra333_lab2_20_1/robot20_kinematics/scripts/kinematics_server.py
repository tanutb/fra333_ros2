#!/usr/bin/python3

import sys
import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import JointState
from robot20_kinematics.kinematics_module import forki,invki
from robot20_kinematics_interfaces.srv import GetPosition
from robot20_kinematics_interfaces.srv import SolveIK



class DummyNode(Node):
    def __init__(self):
        super().__init__('kinematic_server')
        self.flag = False
        self.pub = self.create_publisher(JointState,'/joint_states',10) 
        self.q_services = self.create_service(GetPosition,'/set_joints',self.set_joints_callback)
        self.pos_services = self.create_service(SolveIK,'/solve_ik',self.solve_ik_callback)
        self.timer = self.create_timer(1/10,self.timer_callback)
        self.eff_pos = [0.,0.,0.]
        
        ### HOME CONFIGURATION ###
        self.data = JointState()
        self.data.name = ['joint_1','joint_2','joint_3']
        self.data.position = [0.,0.,0.]
        
    def timer_callback(self):
        #update
        now = self.get_clock().now()
        self.data.header.stamp = now.to_msg()
        self.pub.publish(self.data)
        # self.get_logger().info(f'END-enfector position : {self.position}')

    def set_joints_callback(self,req,res):
        self.data.position = req.joint_state.position
        H = forki(self.data.position)
        self.eff_pos = [H[0][3],H[1][3],H[2][3]]  
        res.pos.x = self.eff_pos[0]
        res.pos.y = self.eff_pos[1]
        res.pos.z = self.eff_pos[2]
        return res
    
    def solve_ik_callback(self,req,res):
        gamma = req.gamma
        a = gamma[0] + gamma[1]
        if (int(a) in [0,2,-2]) and gamma[0] != 0:
            pos = [req.pos.x,req.pos.y,req.pos.z]
            Q = invki(req.gamma[0],req.gamma[1],req.pos.x,req.pos.y,req.pos.z)
            self.flag = Q[1]
            #update 
            self.data.position = Q[0]
            res.joint_state = self.data
            res.flag = Q[1]
            return res
        self.flag = False
        self.data.position = [0.0,0.0,0.0]    
        self.get_logger().info(f'Wrong input')  
        res.joint_state.position = [0.,0.,0.]
        return res
    
def main(args=None):
    rclpy.init(args=args)
    node = DummyNode()
    try:
        while rclpy.ok():
            rclpy.spin_once(node)
    except KeyboardInterrupt:
        print('repeater stopped cleanly')
    except BaseException:
        print('exception in repeater:', file=sys.stderr)
        raise
    finally:
        node.destroy_node()
        rclpy.shutdown() 

if __name__=='__main__':
    main()
