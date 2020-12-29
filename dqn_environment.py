#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Ryan Shim, Gilbert

import math
import numpy
import numpy as np
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node # class for creating node
from rclpy.qos import QoSProfile # class for publishg/subscribing from topics.
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty
import time
from turtlebot3_msgs.srv import Dqn


class DQNEnvironment(Node):
    def __init__(self):
        super().__init__('dqn_environment')

        """************************************************************
        ** Initialise variables
        ************************************************************"""
        self.goal_pose_x = 0.0
        self.goal_pose_y = 0.0
        self.last_pose_x = 0.0
        self.last_pose_y = 0.0
        self.last_pose_theta = 0.0

        self.action_size = 5
        self.done = False
        self.fail = False
        self.succeed = False

        self.goal_angle = 0.0
        self.goal_distance = 1.0
        self.init_goal_distance = 1.0
        self.scan_ranges = []
        self.min_obstacle_distance = 10.0
        self.min_obstacle_angle = 10.0

        self.local_step = 0

        """************************************************************
        ** Initialise ROS publishers and subscribers
        ************************************************************"""
        qos = QoSProfile(depth=10)

        # Initialise publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', qos)

        # Initialise subscribers
        self.goal_pose_sub = self.create_subscription( #goal coordinates 
            Pose,
            'goal_pose',
            self.goal_pose_callback,
            qos)
        self.odom_sub = self.create_subscription( #turtlebot's coordinate from odoometry - look at - turtlebot3_ws/src/turtlebot3/turtlebot3_simulations/turtlebot3_gazebo/models/turtlebot3_waffle_pi
            Odometry,
            'odom',
            self.odom_callback,
            qos)
        self.scan_sub = self.create_subscription( #lidar data from lidar sensor
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile=qos_profile_sensor_data)

        # Initialise client
        self.task_succeed_client = self.create_client(Empty, 'task_succeed')
        self.task_fail_client = self.create_client(Empty, 'task_fail')

        # Initialise servers
        self.dqn_com_server = self.create_service(Dqn, 'dqn_com', self.dqn_com_callback)

    """*******************************************************************************
    ** Callback functions and relevant functions
    *******************************************************************************"""
    def goal_pose_callback(self, msg):
        self.goal_pose_x = msg.position.x
        self.goal_pose_y = msg.position.y
        #if self.succeed == True:
            #time.sleep(0.5)
            #self.init_goal_distance = math.sqrt(
                #(self.goal_pose_x-self.last_pose_x)**2
                #+ (self.goal_pose_y-self.last_pose_y)**2)
            #self.succeed=False
    def odom_callback(self, msg):
        self.last_pose_x = msg.pose.pose.position.x
        self.last_pose_y = msg.pose.pose.position.y
        _, _, self.last_pose_theta = self.euler_from_quaternion(msg.pose.pose.orientation)

        goal_distance = math.sqrt(
            (self.goal_pose_x-self.last_pose_x)**2
            + (self.goal_pose_y-self.last_pose_y)**2)

        path_theta = math.atan2( # angle between tuirtlebot's centre and goal's coordinate
            self.goal_pose_y-self.last_pose_y,
            self.goal_pose_x-self.last_pose_x)

        goal_angle = path_theta - self.last_pose_theta  # theta = phi - omega (see fig in overleaf)
        if goal_angle > math.pi:
            goal_angle -= 2 * math.pi

        elif goal_angle < -math.pi:
            goal_angle += 2 * math.pi

        self.goal_distance = goal_distance
        self.goal_angle = goal_angle

    def scan_callback(self, scan): # subscribes to lidar topic and converts into rl state
        self.scan_ranges = []
        #print("size of ranges"+str(len(scan.ranges)))
        for i in range(len(scan.ranges)):
            if scan.ranges[i] == float('Inf'):
                self.scan_ranges.append(3.5)
            elif np.isnan(scan.ranges[i]):
                self.scan_ranges.append(0)
            else:
                self.scan_ranges.append(scan.ranges[i])
        #print("max of ranges: "+str(max(self.scan_ranges)))
        self.min_obstacle_distance = round(min(self.scan_ranges),2)
        self.min_obstacle_angle = numpy.argmin(self.scan_ranges)

    def get_state(self):
        state = list()
        #state.append(self.scan_ranges)
        for i in range(len(self.scan_ranges)):
            state.append(float(self.scan_ranges[i]))
        state.append(float(self.goal_distance))
        state.append(float(self.goal_angle))
        state.append(float(self.min_obstacle_distance))
        state.append(float(self.min_obstacle_angle))
        self.local_step += 1

        # Succeed
        if self.goal_distance < 0.40:  # unit: m
            print("Goal! :)")
            self.succeed = True
            self.done = True
            self.cmd_vel_pub.publish(Twist())  # robot stop
            self.local_step = 0
            req = Empty.Request()
            while not self.task_succeed_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('service not available, waiting again...')
            self.task_succeed_client.call_async(req)
            #time.sleep(1.0)
        # Fail
        if self.min_obstacle_distance < 0.25:  # unit: m
            print("Collision! :(")
            self.fail = True
            self.done = True
            self.cmd_vel_pub.publish(Twist())  # robot stop
            self.local_step = 0
            req = Empty.Request()
            while not self.task_fail_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('service not available, waiting again...')
            self.task_fail_client.call_async(req)

        if self.local_step == 500:
            print("Time out! :(")
            self.done = True
            self.local_step = 0
            req = Empty.Request()
            while not self.task_fail_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('service not available, waiting again...')
            self.task_fail_client.call_async(req)

        return state

    def reset(self):
        return self.state

    def dqn_com_callback(self, request, response):
        action = request.action
        twist = Twist() # to change turtlebot's pose 
        twist.linear.x = 0.15 # assign linear velocity
        twist.angular.z = ((self.action_size - 1)/2 - action) * 0.75 # translating q values index from fcn to action 
        self.cmd_vel_pub.publish(twist) # send the values to the turtlebot in gaxebo

        response.state = self.get_state() # send state and reward values to agent
        response.reward = self.get_reward(action)
        response.done = self.done



        #self.succeed = False
        if self.done is True:
            self.done = False
            self.succeed=False
            self.fail = False
        if request.init == True :
            self.init_goal_distance = math.sqrt(
                (self.goal_pose_x-self.last_pose_x)**2
                + (self.goal_pose_y-self.last_pose_y)**2)

        return response

    def get_reward(self, action):
        print("intial goal :" +str(self.init_goal_distance))
        print("final goal: "+str(self.goal_distance))
		#Balanced Class
		'''
        yaw_reward = 1 - 2*((math.fabs(self.goal_angle / (math.pi))))
        if yaw_reward >=0:
            angle_reward = 1 * yaw_reward
        elif yaw_reward < 0:
            angle_reward= yaw_reward
        
        if self.goal_distance > (2 * self.init_goal_distance):
            distance_rate = -1.0
        elif self.goal_distance <= (2 * self.init_goal_distance):
            distance_rate= 1 - (math.fabs(self.goal_distance/self.init_goal_distance))
        if distance_rate >=0:
            distance_reward = 1*distance_rate
        elif distance_rate<0:
            distance_reward = distance_rate
        
        print("distance reward :" +str(distance_reward))
        print("angle reward :" +str(angle_reward))
		'''
		
		#Skewed Positive Class
		'''
        yaw_reward = 1 - 2*(math.square(math.fabs(self.goal_angle / (math.pi))))
        if yaw_reward >=0:
            angle_reward = 1 * yaw_reward
        elif yaw_reward < 0:
            angle_reward= yaw_reward
        

        if self.goal_distance > (2 * self.init_goal_distance):
            distance_rate = -1.0
        elif self.goal_distance <= (2 * self.init_goal_distance):
			qwe= math.square(math.fabs(self.goal_distance/self.init_goal_distance))
            distance_rate= 1 - (qwe/2)
        if distance_rate >=0:
            distance_reward = 1*distance_rate
        elif distance_rate<0:
            distance_reward = distance_rate
   
        print("distance reward :" +str(distance_reward))
        print("angle reward :" +str(angle_reward))
		'''
		
		
		#Skewed Negative Class
		'''
        yaw_reward = 1 - 2*(math.sqrt(math.fabs(self.goal_angle / (math.pi))))
        if yaw_reward >=0:
            angle_reward = 2 * yaw_reward
        elif yaw_reward < 0:
            angle_reward= yaw_reward
        

        if self.goal_distance > (2 * self.init_goal_distance):
            distance_rate = -1.0
        elif self.goal_distance <= (2 * self.init_goal_distance):
            distance_rate= 1 - (math.sqrt(2*(math.fabs(self.goal_distance/self.init_goal_distance))))
        if distance_rate >=0:
            distance_reward = 2*distance_rate
        elif distance_rate<0:
            distance_reward = distance_rate
   
        print("distance reward :" +str(distance_reward))
        print("angle reward :" +str(angle_reward))
		'''
		
		
        reward = round(angle_reward , 2) + round(distance_reward,2)
        # + for succeed, - for fail
        if self.succeed:
            reward = 200.00
        elif self.fail:
            reward = - 150.00
        print(reward)

        return reward

    """*******************************************************************************
    ** Below should be replaced when porting for ROS 2 Python tf_conversions is done.
    *******************************************************************************"""
    def euler_from_quaternion(self, quat):
        """
        Converts quaternion (w in last place) to euler roll, pitch, yaw
        quat = [x, y, z, w]
        """
        x = quat.x
        y = quat.y
        z = quat.z
        w = quat.w

        sinr_cosp = 2 * (w*x + y*z)
        cosr_cosp = 1 - 2*(x*x + y*y)
        roll = numpy.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w*y - z*x)
        pitch = numpy.arcsin(sinp)

        siny_cosp = 2 * (w*z + x*y)
        cosy_cosp = 1 - 2 * (y*y + z*z)
        yaw = numpy.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw


def main(args=None):
    rclpy.init(args=args)
    dqn_environment = DQNEnvironment()
    rclpy.spin(dqn_environment)

    dqn_environment.destroy()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
