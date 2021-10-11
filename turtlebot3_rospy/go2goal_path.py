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
    def __init__(self):
        rospy.init_node('turtlebot_ctrl', anonymous=True)
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.odom_subscriber = rospy.Subscriber('/odom', Odometry, self.update_pose)
        self.error_publisher = rospy.Publisher('error', Float32, queue_size=1)
        self.odom = Odometry()
        self.pose = self.odom.pose.pose.position
        self.yaw = 0
        self.rate = rospy.Rate(100)
        self.p = 0
    
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
                vel_msg.linear.x = max(self.linear_velo(goal_pose, P = 0.5), 0.2)
                vel_msg.linear.y = 0
                vel_msg.linear.z = 0

                vel_msg.angular.x = 0
                vel_msg.angular.y = 0
                vel_msg.angular.z = self.angular_velo(goal_pose, P = 0.5)
            self.velocity_publisher.publish(vel_msg)
            self.error_publisher.publish(self.l2_distance(goal_pose))
            self.rate.sleep()
        logmsg = f"reached checkpoint, at x: {round(self.pose.x, 4)} y: {round(self.pose.y, 4)}"
        rospy.loginfo(logmsg)
        
        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0

        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)


def getCircle(x0, y0, r, step):
    x1 = np.arange(x0-r, x0+r, step)
    x2 = np.flip(x1)
    y1 = (2*y0 + np.sqrt(4*y0**2 - 4*(y0**2 + (x1 - x0)**2 - r**2)))/2
    y2 = (2*y0 - np.sqrt(4*y0**2 - 4*(y0**2 + (x1 - x0)**2 - r**2)))/2
    x = np.concatenate((x1, x2))
    x = np.concatenate((x, x[np.newaxis, 0]))
    y = np.concatenate((y1, y2))
    y = np.concatenate((y, y[np.newaxis, 0]))
    x = np.flip(x)
    y = np.flip(y)
    return [x, y]

def getSquare(x0, y0, l, step):
    x1 = np.arange(x0, x0+l+step, step)
    x2 = np.ones(x1.shape[0])*(x0+l)
    x3 = np.arange(x0+l, x0-step, -step)
    x4 = np.ones(x1.shape[0])*x0
    y1 = np.ones(x1.shape[0])*y0
    y2 = np.arange(y0, y0+l+step, step)
    y3 = np.ones(x1.shape[0])*(y0+l)
    y4 = np.arange(y0+l, y0-step, -step)
    x = np.concatenate((x1, x2, x3, x4))
    y = np.concatenate((y1, y2, y3, y4))
    return [x, y]

def getPathInfo():
    while (True):
        pathType = input("Enter type of path (C or c: circle, S or s: square): ")
        if pathType == 'C' or pathType == 'c':
            path_cx = float(input("Enter x of centre: "))
            path_cy = float(input("Enter y of centre: "))
            path_r = float(input("Enter radius of circle: "))
            path_step = float(input("Enter step: "))
            [goal_x, goal_y] = getCircle(path_cx, path_cy, path_r, path_step)
            break
        elif pathType == 'S' or pathType == 's':
            path_cx = float(input("Enter x of starting point: "))
            path_cy = float(input("Enter y of starting point: "))
            path_l = float(input("Enter length of square: "))
            path_step = float(input("Enter step: "))
            [goal_x, goal_y] = getSquare(path_cx, path_cy, path_l, path_step)
            break
        else:
            print("Please enter a valid path type")
    return [goal_x, goal_y]



if __name__=='__main__':
    try:
        getSquare(0,0,1,1)
        if len(sys.argv) == 6:
            if (sys.argv[1] == 'c' or sys.argv[1] == 'C'):
                path_cx = float(sys.argv[2])
                path_cy = float(sys.argv[3])
                path_r = float(sys.argv[4])
                path_step = float(sys.argv[5])
                [goal_x, goal_y] = getCircle(path_cx, path_cy, path_r, path_step)
            if (sys.argv[1] == 's' or sys.argv[1] == 'S'):
                path_cx = float(sys.argv[2])
                path_cy = float(sys.argv[3])
                path_l = float(sys.argv[4])
                path_step = float(sys.argv[5])
                [goal_x, goal_y] = getSquare(path_cx, path_cy, path_l, path_step)
        else:
            [goal_x, goal_y] = getPathInfo()
        goal_tol = 0.1
        x = TurtleBot()
        for i, j in zip(goal_x, goal_y):
            rospy.loginfo(f"next checkpoint is {round(i, 4)}, {round(j, 4)}")
            x.move2goal(i, j, goal_tol)
        rospy.loginfo("reached all checkpoints")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
