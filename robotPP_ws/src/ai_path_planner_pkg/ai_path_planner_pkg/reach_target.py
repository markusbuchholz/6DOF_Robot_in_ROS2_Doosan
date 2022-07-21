import rclpy
from rclpy.duration import Duration
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

#Reinforcement Learning
from .robot_brain import *
from ik_interface.srv import XYZJoints         
import sys



class TrajectoryActionClient(Node):

    def __init__(self):

        super().__init__('points_publisher_node_action_client')
        self.action_client = ActionClient(
            self, FollowJointTrajectory, '/joint_trajectory_controller/follow_joint_trajectory')
        self.cli = self.create_client(XYZJoints, 'compute_joints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = XYZJoints.Request()

    def send_request(self, position):

        self.req.x = float(position[0])
        self.req.y = float(position[1])
        self.req.z = float(position[2])
        self.future = self.cli.call_async(self.req)
        

    def send_goal(self, pos):
     
        points = []
        i = 0
        for response in pos:
            print ("joints target :", response.j1, " : ", response.j2)

            point_msg = JointTrajectoryPoint()
            point_msg.positions = [response.j1,  response.j2, response.j3, response.j4, response.j5, response.j6]
            point_msg.time_from_start = Duration(seconds=2.0+(2.0*i)).to_msg()
            points.append(point_msg)

            i = i + 1


        joint_names = ['joint1', 'joint2',
                    'joint3', 'joint4', 'joint5', 'joint6']
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.goal_time_tolerance = Duration(
            seconds=1, nanoseconds=0).to_msg()
        goal_msg.trajectory.joint_names = joint_names
        goal_msg.trajectory.points = points
        print("--> 2")
        self.action_client.wait_for_server()
        print("--> 3")
        self.send_goal_future = self.action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        self.send_goal_future.add_done_callback(self.goal_response_callback)
            

    def goal_response_callback(self, future):
        self.get_logger().info('====1======')
        print("1. goal_response_callback")
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected ')
            return

        self.get_logger().info('Goal accepted')

        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        self.get_logger().info('====2======')
        print("2. get_result_callback")
        result = future.result().result
        self.get_logger().info('Result: '+str(result))
        #rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        self.get_logger().info('====3======')
        print("3. feedback_callback")
        feedback = feedback_msg.feedback


def main(args=None):
    waypoints = run_brain()
    print("robot brain =>", waypoints)
    print("type =>", type(waypoints))
    print("value =>", waypoints[0][0])
   
    brain = waypoints
    targets = []

    #in order to demonstrate basics
    #we use only 3 waypoints
  
    Z = 0.6
    for ii in waypoints:
        if ii == (0,1):
            targets.append([0.5, 0.5, Z])
        if ii == (0,0):
            targets.append([0.5, 0.5, Z])
        if ii == (0,2):
            targets.append([0.05, 0.05, Z+0.3])
        if ii == (1,2):
            targets.append([0.05, 0.05, Z+0.3])
        if ii == (2,1):
            targets.append([0.5, -0.5, Z])
        if ii == (2,2):
            targets.append([0.5, -0.5, Z])

            
    print("==BRAIN==")
    print(targets) 

    positions = []

    for pos in targets:
        print("target i :", pos)
        positions.append(pos)


    pos_joints = []

    for pos in positions:
        rclpy.init()
        action_client = TrajectoryActionClient()
        action_client.send_request(pos)
        rclpy.spin_once(action_client)
        response = action_client.future.result()
        if action_client.future.done():
            pos_joints.append(response)
            print("::: ",response.j1, " : ",  response.j2," : ", response.j3, " : ",response.j4," : ", response.j5," : ", response.j6 )
            rclpy.shutdown()


    rclpy.init()
    action_client = TrajectoryActionClient()
    action_client.send_goal(pos_joints)
    rclpy.shutdown()




if __name__ == '__main__':
    main()
