#!/usr/bin/env python

import rospy
import actionlib
from visualization_msgs.msg import Marker
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
import time
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.markers as markers

map_data = OccupancyGrid()
odom = Odometry
bot_x = 0
bot_y = 0

def map_callback(data):
    global map_data
    map_data = data

def update_pose(data):
    global odom
    odom = data
    bot_x = odom.pose.pose.position.x
    bot_y = odom.pose.pose.position.y

def get_map():
    odom_subscriber = rospy.Subscriber('/odom', Odometry, update_pose)
    map_client = rospy.Subscriber("/map", OccupancyGrid, map_callback)
    while map_data.header.seq<1 or len(map_data.data)<1:
        pass
    get_frontier()


def get_frontier():
    # marker_publisher = rospy.Publisher('shapes', Marker, queue_size=10)
    data = map_data.data
    w = map_data.info.width
    h = map_data.info.height
    resolution = map_data.info.resolution
    ori_x = map_data.info.origin.position.x
    ori_y = map_data.info.origin.position.y
    # print(bot_x, bot_y)
    data_np = np.asarray(data)
    for i in range(0, len(data)):
        if data_np[i] == -1:
            data_np[i] = 255
        elif data_np[i] == 100:
            data_np[i] = 0
        elif data_np[i] == 0:
            data_np[i] = 127
    data_np = np.reshape(data_np, [h, w]);
    print(np.unique(data_np))
    plt.figure()
    plt.imshow(data_np, cmap='gray', vmin = 0, vmax = 255)
    plt.plot(int((bot_x - ori_x)/resolution), int((bot_y - ori_y)/resolution), 'bo--', linewidth=2, markersize=12)
    plt.show()

def movebase_client(x_goal, y_goal):
    # Create an action client "move_base"
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
 
    # Waits until the action server has started
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = float(x_goal)
    goal.target_pose.pose.position.y = float(y_goal) 
    # No rotation of the mobile base frame w.r.t. map frame
    goal.target_pose.pose.orientation.w = 1.0

    # Sends the goal to the action server.
    client.send_goal(goal)
    # Waits for the server to finish performing the action.
    # If the result doesn't arrive, assume the Server is not available
    if not client.wait_for_result:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
    # Result of executing the action
        return client.get_result()   

# If the python node is executed as main process (sourced directly)
if __name__ == '__main__':
    try:
        #Initializes a rospy node to let the SimpleActionClient publish and subscribe
        rospy.init_node('movebase_client_py')
        # x_goal = 0
        # y_goal = 0
        # result = movebase_client(x_goal, y_goal)
        # if result:
        #     rospy.loginfo("Goal execution done!")
        # while True:
        get_map()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")