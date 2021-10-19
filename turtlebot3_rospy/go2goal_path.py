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
import threading
import sys
import signal

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
                vel_msg.linear.x = max(self.linear_velo(goal_pose, P = 0.5), 0.3)
                vel_msg.linear.y = 0
                vel_msg.linear.z = 0

                vel_msg.angular.x = 0
                vel_msg.angular.y = 0
                vel_msg.angular.z = self.angular_velo(goal_pose, P = 0.5)
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
    theta = -4*np.ones(x.size)
    goal = list(zip(x, y, theta))
    return goal

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
    theta = -4*np.ones(x.size)
    goal = list(zip(x, y, theta))
    return goal 

global goal
goal = []
global goal_tol
goal_tol = 0.1

def input_thread(lock):
    new_goal = []
    while True:
        while True:
            # pathLine = input("Generate path: ").split(' ')
            pathLine = input().split(' ')
            if len(pathLine) == 5 and (pathLine[0] == 'C' or pathLine[0] == 'c'):
                path_cx = float(pathLine[1])
                path_cy = float(pathLine[2])
                path_r = float(pathLine[3])
                path_step = float(pathLine[4])
                new_goal = getCircle(path_cx, path_cy, path_r, path_step)
                break
            elif len(pathLine) == 5 and (pathLine[0] == 'S' or pathLine[0] == 's'):
                path_cx = float(pathLine[1])
                path_cy = float(pathLine[2])
                path_l = float(pathLine[3])
                path_step = float(pathLine[4])
                new_goal = getSquare(path_cx, path_cy, path_l, path_step)
                break
            elif len(pathLine) == 4 and pathLine[0] == 'pt':
                goal_x = float(pathLine[1])
                goal_y = float(pathLine[2])
                goal_theta= float(pathLine[3])
                if (abs(goal_theta) > 3.14):
                    goal_theta = -4
                new_goal = list(zip([goal_x], [goal_y], [goal_theta]))
                break
            elif len(pathLine) == 3 and pathLine[0] == 'pt':
                goal_x = float(pathLine[1])
                goal_y = float(pathLine[2])
                goal_theta = -4
                new_goal = list(zip([goal_x], [goal_y], [goal_theta]))
                break
            elif len(pathLine) == 1 and pathLine[0] == 'clear':
                lock.acquire()
                goal.clear()
                lock.release()
                rospy.loginfo("cleared all goals")
                break
            else:
                print("Invalid path")
        # return new_goal
        if pathLine[0] != 'clear':
            lock.acquire()
            for i in new_goal:
                goal.append(i)
            lock.release()


def move_bot_thread(bot_name, bot, goal, lock):
    print(f"Thread {bot_name} starts")
    while True:
        next_goal = tuple()
        if len(goal) != 0:
            lock.acquire()
            next_goal = goal[0]
            goal.pop(0)
            lock.release()
        if next_goal != tuple():
            if (next_goal[2] == -4):
                rospy.loginfo(f"next checkpoint is {round(next_goal[0], 4)}, {round(next_goal[1], 4)}")
                bot.move2goal(next_goal[0], next_goal[1], goal_tol)
            else:
                rospy.loginfo(f"next checkpoint is {round(next_goal[0], 4)}, {round(next_goal[1], 4)}, theta = {next_goal[2]}")
                bot.move2goal_oriented(next_goal[0], next_goal[1], next_goal[2], goal_tol)

def signal_handler(sig, frame):
    print('Program ending....')
    sys.exit()


if __name__=='__main__':
    usage_msg = """Usage:
    For points, use cmd (theta can be empty): pt <x> <y> <theta>
    For circle, use cmd: c <center x> <center y> <radius> <step>
    For square, use cmd: s <corner x> <corner y> <length> <step>
    To clear all inputs, use cmd: clear
    These cmd can be added while the robot is executing the previous cmd
    To exit program, press Ctrl C"""
    print(usage_msg)
    lock = threading.Lock()
    if len(sys.argv) == 6:
        goal_theta = -4
        if (sys.argv[1] == 'c' or sys.argv[1] == 'C'):
            path_cx = float(sys.argv[2])
            path_cy = float(sys.argv[3])
            path_r = float(sys.argv[4])
            path_step = float(sys.argv[5])
            new_goal = getCircle(path_cx, path_cy, path_r, path_step)
            goal.append(new_goal)
        elif (sys.argv[1] == 's' or sys.argv[1] == 'S'):
            path_cx = float(sys.argv[2])
            path_cy = float(sys.argv[3])
            path_l = float(sys.argv[4])
            path_step = float(sys.argv[5])
            new_goal = getSquare(path_cx, path_cy, path_l, path_step)
            goal.append(new_goal)
    elif (len(sys.argv) == 5 and sys.argv[1] == 'pt'):
        goal_x = float(sys.argv[2])
        goal_y = float(sys.argv[3])
        goal_theta = float(sys.argv[4])
        new_goal = list(zip([goal_x], [goal_y], [goal_theta]))
        goal.append(new_goal)
    elif (len(sys.argv) == 4 and sys.argv[1] == 'pt'):
        goal_x = float(sys.argv[2])
        goal_y = float(sys.argv[3])
        new_goal = list(zip([goal_x], [goal_y], [-4]))
        goal.append(new_goal)
    x = TurtleBot()
    threadBot = threading.Thread(target=move_bot_thread, args=("turtlebot", x, goal, lock), daemon=True)
    threadBot.start()
    threadInput = threading.Thread(target=input_thread, args=(lock, ), daemon=True)
    threadInput.start()
    while (threadBot.is_alive()):
        signal.signal(signal.SIGINT, signal_handler)