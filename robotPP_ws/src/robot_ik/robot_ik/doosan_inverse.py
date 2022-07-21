#!/usr/bin/env python3

from .RobotSerial import *
import numpy as np
from math import pi

#############
from ik_interface.srv import XYZJoints 
import rclpy
from rclpy.node import Node
############





class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(XYZJoints, 'compute_joints', self.computeIK)        # CHANGE

    def computeIK(self, request, response):
        np.set_printoptions(precision=3, suppress=True)
        #d, a, alfa, theta
        dh_params_doosan = np.array([[0.195, 0.0, -0.5 * pi, 0.0],
                              [0.0, 0.560, 0.0, -0.5 * pi],
                              [0.039, 0.0, 0.5* pi, 0.5 * pi],
                              [0.516, 0.0, -0.5 * pi, 0.0],
                              [0.0, 0.0, 0.5 * pi, 0.0],
                              [0.124, 0.0, 0.0, 0.0]])

        robot = RobotSerial(dh_params_doosan)

        # =====================================
        # inverse
        # =====================================
        #x, y, z = input("xyz : ").split()

        #xyz = np.array([[0.4], [0.3], [0.2]])
        xyz = np.array([[request.x], [request.y], [request.z]])
        orientation = np.array([0.0, pi/2, 0.0])
        end = Frame.from_euler_3(orientation, xyz)
        robot.inverse(end)

        print("inverse is successful: {0}".format(robot.is_reachable_inverse))
      
        print("axis values: \n{0}".format(robot.axis_values))
  

        response.j1 =robot.axis_values[0] #* 180.0/pi 
        response.j2 =robot.axis_values[1] #* 180.0/pi 
        response.j3 =robot.axis_values[2] #* 180.0/pi 
        response.j4 =robot.axis_values[3] #* 180.0/pi 
        response.j5 =robot.axis_values[4] #* 180.0/pi 
        response.j6 =robot.axis_values[5] #* 180.0/pi         
        
        self.get_logger().info('Incoming request\nx: %f y: %f z: %f' % (request.x, request.y, request.z)) # CHANGE

        return response

def main(args=None):
    rclpy.init(args=args)

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == "__main__":
    main()