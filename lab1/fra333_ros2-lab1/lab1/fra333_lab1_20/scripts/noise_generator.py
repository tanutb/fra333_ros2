#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from lab1_interfaces.srv import SetNoise
import numpy as np


# import all other neccesary libraries here

import sys

class NoiseGenerator(Node):

    def __init__(self):
        super().__init__("noise_generator")
        # get the rate from argument or default
        if len(sys.argv)>=2: 
            self.rate = float(sys.argv[1])
        else:
            self.rate = 5.0
        # add codes here
        self.test = sys.argv
        self.noise_publisher = self.create_publisher(Float64,'noise',10)
        self.set_noise_service = self.create_service(SetNoise,'set_noise',self.set_noise_callback)     
        self.timer = self.create_timer(1/self.rate,self.timer_callback)
        
        # additional attributes
        self.mean = 0.0
        self.variance = 1.0
        self.get_logger().info(f'Starting {self.get_namespace()}/{self.get_name()} with the default parameter. mean: {self.mean}, variance: {self.variance} ,self.rate: {self.rate} test : {self.test }')
        print(self.test)
    
    def set_noise_callback(self,request:SetNoise.Request,response:SetNoise.Response):
        self.mean = request.mean.data
        self.variance = request.variance.data
        self.get_logger().info(f'request\n mean: {self.mean} variance: {self.variance}')

        # Call request
        # self.result = self.client.call_async(request)

        
        return response
    
    def timer_callback(self):
        noise = Float64()
        noise.data = np.random.normal(self.mean, self.variance ** 0.5)
        self.noise_publisher.publish(noise)
        


def main(args=None):
    rclpy.init(args=args)
    node = NoiseGenerator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
