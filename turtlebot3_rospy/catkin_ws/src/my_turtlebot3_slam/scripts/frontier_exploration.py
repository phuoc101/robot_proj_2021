#!/usr/bin/env python

import rospy
import actionlib
from visualization_msgs.msg import Marker
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
import time
import numpy as np
from numpy.linalg import norm
import matplotlib.pyplot as plt
import matplotlib.markers as markers
from skimage.measure import find_contours

map_data = OccupancyGrid()
odom = Odometry
bot_x = 0
bot_y = 0
resolution = 0
ori_x = 0
ori_y = 0

def map_callback(data):
    global map_data
    global resolution, ori_x, ori_y, w, h
    map_data = data
    resolution = map_data.info.resolution
    ori_x = map_data.info.origin.position.x
    ori_y = map_data.info.origin.position.y

def update_pose(data):
    global odom, bot_x, bot_y
    odom = data
    bot_x = odom.pose.pose.position.x
    bot_y = odom.pose.pose.position.y

def get_map():
    odom_subscriber = rospy.Subscriber('/odom', Odometry, update_pose)
    map_client = rospy.Subscriber("/map", OccupancyGrid, map_callback)
    while map_data.header.seq<1 or len(map_data.data)<1:
        pass
    get_frontier()

def bot_index(x, y):
    return np.array([int((x - ori_x)/resolution), int((y - ori_y)/resolution)])

def to_map_coord(contour):
    for index in range(len(contour)):
        contour[index][0] = round(contour[index][0] * resolution + ori_x, 2)
        contour[index][1] = round(contour[index][1] * resolution + ori_y, 2)
    return contour

def get_frontier():
    data = map_data.data
    # marker_publisher = rospy.Publisher('shapes', Marker, queue_size=10)
    # print(bot_x, bot_y)
    w = map_data.info.width
    h = map_data.info.height
    data_np = np.asarray(data)
    data_np = np.reshape(data_np, [h, w]);
    bot_idx = bot_index(bot_x, bot_y)
    ## for visualizing ####
    # for i in range(0, data_np.shape[0]):
    #     for j in range(0, data_np.shape[1]):
    #         if data_np[i,j] == -1:
    #             data_np[i,j] = 255
    #         elif data_np[i,j] == 100:
    #             data_np[i,j] = 0
    #         elif data_np[i,j] == 0:
    #             data_np[i,j] = 127
    fig, ax = plt.subplots()
    ax.imshow(data_np, cmap='gray', vmin = -1, vmax = 100)
    ax.plot(bot_idx[0], bot_idx[1], 'bo--', linewidth=2, markersize=12)
    # unknowns
    contour_m1 = find_contours(data_np, -1, fully_connected='high')
    # edges
    contour_0 = find_contours(data_np, 1, fully_connected='high')
    for contour in contour_0:
        ax.plot(contour[:, 1], contour[:, 0], linewidth=2)
    contour_m1 = np.concatenate(contour_m1, axis=0)
    contour_0 = np.concatenate(contour_0, axis=0)
    
    contour_0 = to_map_coord(contour_0)
    contour_m1 = to_map_coord(contour_m1)

    contour_0 = np.asarray(contour_0)
    contour_m1 = np.asarray(contour_m1)

    # convert contour np arrays into sets
    set_0 = set([tuple(x) for x in contour_0])
    set_m1 = set([tuple(x) for x in contour_m1])

    candidates = set_m1.difference(set_0)

    #### uncomment for visualization
    for can in candidates:
        ax.plot((can[1]-ori_x)/resolution, (can[0]-ori_y)/resolution, 'rx', linewidth=1, markersize=1)

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