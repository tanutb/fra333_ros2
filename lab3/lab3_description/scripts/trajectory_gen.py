#!/usr/bin/python3
import numpy as np
import rclpy
import sys, os, yaml
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import Imu

from trajectory_msgs.msg import JointTrajectory , JointTrajectoryPoint

from lab3_description.dummy_module import *

class Trajectory_publisher(Node):

    def __init__(self):
        super().__init__('trajectory_publsiher_node')
        publish_topic = "/joint_trajectory_position_controller/joint_trajectory"
        self.trajectory_publihser = self.create_publisher(JointTrajectory,publish_topic, 10)
        self.sub = self.create_subscription(Imu,"/Imu_arduino" ,self.sub_callback, 10)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.joints = ['joint_1','joint_2','joint_3']

        self.calib_imu = read_json()
        self.Flag_calib = check_flag(self.calib_imu )
        self.imu_dict = {"angular_velocity":[],"linear_acceleration":[]}    
        
        self.angular_velocity = np.array([0.,0.,0.])
        self.linear_acceleration  = np.array([0.,0.,0.])
        self.point = JointTrajectoryPoint()
        
        self.theta = np.array([0.,0.,0.])
        self.q = np.array([0.,0.,0.])       
        self.position_xyz = np.array([0.,0.,0.])
        
    def sub_callback(self,msg):
        self.angular_velocity = [msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z]
        self.linear_acceleration = [msg.linear_acceleration.x,msg.linear_acceleration.y,msg.linear_acceleration.z]   
        
        
    def setgoal(self,req,res):
        self.goal_positions = list(req.g)
        return res
        

    def timer_callback(self):
        if self.Flag_calib :
            
            sd = calibration_IMU(self.calib_imu)    
            # print(mean[0:3])
            self.angular_velocity_new = np.array([0.,0.,0.])
            self.angular_velocity_new[0] = self.angular_velocity[0] - sd[0]
            self.angular_velocity_new[1] = self.angular_velocity[1] + sd[1]
            self.angular_velocity_new[2] = self.angular_velocity[2] + sd[2]
            self.linear_acceleration_new = np.array(self.linear_acceleration) - np.array(sd[3::])
            self.theta += self.angular_velocity_new * 0.1
            
            P = forki(self.theta)
            [x,y,z] = [P[0][3],P[1][3],P[2][3]]
            
            for i in [[1,1],[1,-1],[-1,1],[-1,-1]]:
                q ,flag = invki(i[0],i[1],x,y,z)
                if flag and z > 0:
                    self.q = q 
                    self.position_xyz = [x,y,z]
                    
                    break
                            
            print(self.theta)
            
            bazu_trajectory_msg = JointTrajectory()
            bazu_trajectory_msg.joint_names = self.joints
            ## creating a point
            self.point.positions = list(self.q)
            self.point.time_from_start = Duration(sec=1)
            ## adding newly created point into trajectory message
            bazu_trajectory_msg.points.append(self.point)
            self.trajectory_publihser.publish(bazu_trajectory_msg)
            
        else : 
            if len(self.imu_dict["angular_velocity"]) != 1000:
                print(len(self.imu_dict["angular_velocity"]))
                self.imu_dict["angular_velocity"].append(self.angular_velocity)
                self.imu_dict["linear_acceleration"].append(self.linear_acceleration)
            elif len(self.imu_dict["angular_velocity"]) == 1000:
                self.get_logger().info("COMPLETE")             
                write_json(self.imu_dict)
                self.calib_imu = read_json()
                self.Flag_calib = check_flag(self.calib_imu)
 
def main(args=None):
    rclpy.init(args=args)
    joint_trajectory_object = Trajectory_publisher()
    try:
        while rclpy.ok():
            rclpy.spin_once(joint_trajectory_object)
    except KeyboardInterrupt:
        print('repeater stopped cleanly')
    except BaseException:
        print('exception in repeater:', file=sys.stderr)
        raise
    finally:
        joint_trajectory_object.destroy_node()
        rclpy.shutdown() 


if __name__ == '__main__':
    main()