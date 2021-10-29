import rospy
import tf2_ros
from math import pow, atan2, sqrt
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
import time
import sys
import numpy as np


class TurtleBot:
    def __init__(self, name):
        rospy.init_node('turtlebot_ctrl', anonymous=True)
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.odom_subscriber = rospy.Subscriber('/odom', Odometry, self.update_pose)
        self.error_publisher = rospy.Publisher('error', Float32, queue_size=1)
        self.odom = Odometry()
        self.pose = self.odom.pose.pose.position
        self.yaw = 0
        self.rate = rospy.Rate(50)
        self.p = 0
        rospy.loginfo(f"{name} initialized")
    
    def update_pose(self, data):
        self.curr_time = data.header.stamp
        self.odom = data
        self.pose = data.pose.pose.position
        self.ori = data.pose.pose.orientation
        self.pose_list = [self.pose.x, self.pose.y, self.pose.z]
        self.ori_list = [self.ori.x, self.ori.y, self.ori.z, self.ori.w]
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion(self.ori_list)
    
    def l2_distance(self, goal_pose):
        return sqrt(pow((goal_pose.pose.pose.position.x - self.pose.x), 2) +
                    pow((goal_pose.pose.pose.position.y - self.pose.y), 2))

    def linear_velo(self, goal_pose, P = 0.1):
        velo = P * self.l2_distance(goal_pose)
        return velo
    
    def steering_angle(self, goal_pose):
        angle = atan2((goal_pose.pose.pose.position.y - self.pose.y), (goal_pose.pose.pose.position.x - self.pose.x))
        return angle    

    def angular_velo(self, goal_pose, P = 2):
        steer = self.steering_angle(goal_pose)
        yaw = self.yaw
        if (abs(steer - yaw) > np.pi):
            if steer < 0:
                steer = steer + 2*np.pi
            elif yaw < 0:
                yaw = yaw + 2*np.pi
        omega = P * (steer - yaw)
        if abs(omega) < 0.1:
            if omega >= 0:
                omega = 0.1  
            else: 
                omega = -0.1
        elif abs(omega) > 1:
            if omega >= 0:
                omega = 1
            else: 
                omega = -1
        return omega

    def rot(self, goal_theta, P = 1):
        return P*(goal_theta - self.yaw)

    def move2goal(self, goal_x, goal_y, goal_tol):
        goal_pose = Odometry()
        goal_pose.pose.pose.position.x =goal_x 
        goal_pose.pose.pose.position.y = goal_y
        tolerance = goal_tol

        vel_msg = Twist()

        while self.l2_distance(goal_pose) > tolerance :
            if abs(self.steering_angle(goal_pose) - self.yaw) >= tolerance:
                vel_msg.linear.x = 0 
                vel_msg.linear.y = 0
                vel_msg.linear.z = 0

                vel_msg.angular.x = 0
                vel_msg.angular.y = 0
                vel_msg.angular.z = self.angular_velo(goal_pose)
            else:
                vel_msg.linear.x = max(self.linear_velo(goal_pose, P = 0.5), 0.3)
                vel_msg.linear.y = 0
                vel_msg.linear.z = 0

                vel_msg.angular.x = 0
                vel_msg.angular.y = 0
                vel_msg.angular.z = self.angular_velo(goal_pose, P = 0.3)
            self.velocity_publisher.publish(vel_msg)
            self.error_publisher.publish(self.l2_distance(goal_pose))
            self.rate.sleep()
        
        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0

        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)

        logmsg = f"reached checkpoint, at x: {round(self.pose.x, 4)} y: {round(self.pose.y, 4)}"
        rospy.loginfo(logmsg)

    def move2goal_oriented(self, goal_x, goal_y, goal_theta, goal_tol):
        goal_pose = Odometry()

        time.sleep(1)
        goal_pose.pose.pose.position.x =goal_x 
        goal_pose.pose.pose.position.y = goal_y
        tolerance = goal_tol
        vel_msg = Twist()

        while self.l2_distance(goal_pose) > tolerance :
            if abs(self.steering_angle(goal_pose) - self.yaw) >= tolerance:
                vel_msg.linear.x = 0 
                vel_msg.linear.y = 0
                vel_msg.linear.z = 0

                vel_msg.angular.x = 0
                vel_msg.angular.y = 0
                vel_msg.angular.z = self.angular_velo(goal_pose)
            else:
                vel_msg.linear.x = max(self.linear_velo(goal_pose, P = 0.3), 0.2)
                vel_msg.linear.y = 0
                vel_msg.linear.z = 0

                vel_msg.angular.x = 0
                vel_msg.angular.y = 0
                vel_msg.angular.z = self.angular_velo(goal_pose, P = 0.3)
            self.velocity_publisher.publish(vel_msg)
            self.error_publisher.publish(self.l2_distance(goal_pose))
            self.rate.sleep()
        while (abs(self.yaw - goal_theta) > tolerance):
            vel_msg.linear.x = 0
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0

            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = self.rot(goal_theta, P = 1)

            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()

        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0

        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)

        logmsg = f"reached checkpoint, at x: {round(self.pose.x, 4)} y: {round(self.pose.y, 4)}"
        rospy.loginfo(logmsg)
