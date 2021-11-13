from Turtlebot import *
from getpath import getCircle, getSquare
import threading
import sys
import signal
import turtle

global goal
goal = []
global goal_tol
goal_tol = 0.1

def input_thread(bot, lock):
    print("Input thread running")
    new_goal = []
    while True:
        while True:
            cmdLine = str(turtle.textinput("Command window", "Input command"))
            cmdLine = cmdLine.lower()
            pathLine = cmdLine.split(' ')
            if len(pathLine) == 5 and (pathLine[0] == 'c'):
                path_cx = float(pathLine[1])
                path_cy = float(pathLine[2])
                path_r = float(pathLine[3])
                path_step = float(pathLine[4])
                new_goal = getCircle(path_cx, path_cy, path_r, path_step)
                break
            elif len(pathLine) == 5 and (pathLine[0] == 's'):
                path_cx = float(pathLine[1])
                path_cy = float(pathLine[2])
                path_l = float(pathLine[3])
                path_step = float(pathLine[4])
                new_goal = getSquare(path_cx, path_cy, path_l, path_step)
                break
            elif len(pathLine) == 4 and pathLine[0] == 'pt':
                goal_x = float(pathLine[1])
                goal_y = float(pathLine[2])
                goal_theta= float(pathLine[3]) % 3.14
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
                rospy.loginfo("cleared all goals, please enter new inputs")
                break
            elif len(pathLine) == 1 and pathLine[0] == 'quit':
                print('Program ended by user')
                return
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
            if len(goal) == 0:
                rospy.loginfo("Goal list emptied")
            lock.release()
        if next_goal != tuple():
            if (next_goal[2] == -4):
                rospy.loginfo(f"next checkpoint is {round(next_goal[0], 4)}, {round(next_goal[1], 4)}")
                bot.move2goal(next_goal[0], next_goal[1], goal_tol)
            else:
                rospy.loginfo(f"next checkpoint is {round(next_goal[0], 4)}, {round(next_goal[1], 4)}, theta = {next_goal[2]}")
                bot.move2goal_oriented(next_goal[0], next_goal[1], next_goal[2], goal_tol)

def signal_handler(sig, frame):
    print('Program ended by user')
    sys.exit()


if __name__=='__main__':
    usage_msg = """Usage:
    For points, use cmd (theta is in radian, ignored when left empty): pt <x> <y> <theta>
    For circle, use cmd: c <center x> <center y> <radius> <step>
    For square, use cmd: s <corner x> <corner y> <length> <step>
    To clear all inputs, use cmd: clear
    These cmd can be added while the robot is executing the previous cmd
    To exit program, press Ctrl C or enter cmd quit"""
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
        goal.append(new_goal[0])
    elif (len(sys.argv) == 4 and sys.argv[1] == 'pt'):
        goal_x = float(sys.argv[2])
        goal_y = float(sys.argv[3])
        new_goal = list(zip([goal_x], [goal_y], [-4]))
        goal.append(new_goal[0])
    x = TurtleBot("turtlebot3")
    threadBot = threading.Thread(target=move_bot_thread, args=("turtlebot", x, goal, lock))
    threadBot.daemon = True
    threadBot.start()
    threadInput = threading.Thread(target=input_thread, args=(x, lock, ))
    threadInput.daemon = True
    threadInput.start()
    while threadInput.is_alive():
        signal.signal(signal.SIGINT, signal_handler)
