#!/usr/bin/python3
import numpy as np
import rclpy
import sys, os, yaml
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from std_msgs.msg import Float64MultiArray,Float64
from sensor_msgs.msg import JointState
from lab4_robot_control.module import *


class generator(Node):
    
    def __init__(self):
        super().__init__('trajectory_generator_node')

        #Sub from Scheduler
        self.Pi_sub = self.create_subscription(Float64MultiArray,"/Pi" ,self.Pi_sub_callback, 10)
        self.Pf_sub = self.create_subscription(Float64MultiArray,"/Pf" ,self.Pf_sub_callback, 10)
        self.T_sub = self.create_subscription(Float64,"/T" ,self.T_sub_callback, 10)
        #Pub to tracker
        self.pub = self.create_publisher(JointState,"/reference/joint_states",10)
        
        
        # self.hi = self.create_publisher(Float64MultiArray,"/HI",10)
        
        self.timer = self.create_timer(1/10,self.timer_callback)
        # modify these dummy
        self.q_r = np.array([0.,0.,0.])
        self.q_r_dot = np.array([0.,0.,0.])
        self.dt = 0.1
        self.t = 0.
        self.T = 5.
        self.p_init =  np.array([0.3099999,0.,0.13499999])
        self.p_final = np.array([0.3099999,0.,0.13499999])

    def Pi_sub_callback(self,msg:Float64MultiArray):
        self.p_init = np.array(msg.data)
        #Check if p_init change to reset t
            
    def Pf_sub_callback(self,msg:Float64MultiArray):
        self.p_final = np.array(msg.data)
        
    def T_sub_callback(self,msg:Float64):       
        self.T = msg.data
        
    def quintic_trajectory(self,T,dt):
        p_init = 0
        p_final = 1
        a0 = p_init
        a1 = 0
        a2 = 0*0.5
        a3 = -(20*p_init - 20*p_final)/(2*(T**3))
        a4 = (30*p_init - 30*p_final)/(2*(T**4))
        a5 = -(12*p_init - 12*p_final)/(2*(T**5))

        if(dt<=T):
            ref_p = (a0)+(a1*dt)+(a2*dt**2)+(a3*dt**3)+(a4*dt**4)+(a5*dt**5)
            ref_p_dot = (a1)+(2*a2*dt)+(3*a3*dt**2)+(4*a4*dt**3)+(5*a5*dt**4)
        else:

            ref_p = (a0)+(a1*T)+(a2*T**2)+(a3*T**3)+(a4*T**4)+(a5*T**5)
            ref_p_dot = 0
        return ref_p , ref_p_dot

    def timer_callback(self):
        if(np.array_equal(self.p_init, self.p_init)!=True):
            self.t = 0.
        #Create p and p_dot reference
        ref_p , ref_p_dot = self.quintic_trajectory(self.T,self.t)
        self.t += self.dt
        #Recieve from sub via_point_callback and change to np array
        p_init = np.array(self.p_init)
        p_final = np.array(self.p_final)
        
        #Linear Interpolation
        p_r = (1-ref_p)*p_init + ref_p * (p_final)
        p_r_dot = ref_p_dot * (p_final-p_init)
        
        print(p_r)
        #Inverse position
        Gamma = [1,-1]
        self.q_r = IPK(Gamma[0],Gamma[1],p_r[0],p_r[1],p_r[2])
        #Inverse Velocity
        self.q_r_dot = IVK(self.q_r[0],p_r_dot)
        #pub to tracker to PID
        pub_msg = JointState()
        pub_msg.position = list(self.q_r[0])
        pub_msg.velocity = list(self.q_r_dot)

        self.pub.publish(pub_msg)
        
        
        # m = Float64MultiArray()
        # m.data = list(self.p_final)
        # self.hi.publish(m)

def main(args=None):
    rclpy.init(args=args)
    trajectory_generator = generator()
    try:
        while rclpy.ok():
            rclpy.spin_once(trajectory_generator)
    except KeyboardInterrupt:
        print('repeater stopped cleanly')
    except BaseException:
        print('exception in repeater:', file=sys.stderr)
        raise
    finally:
        trajectory_generator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()