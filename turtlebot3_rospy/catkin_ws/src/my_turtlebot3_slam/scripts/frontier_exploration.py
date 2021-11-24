#!/usr/bin/env python

import rospy
import actionlib
from visualization_msgs.msg import Marker, MarkerArray
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry, OccupancyGrid
from std_msgs.msg import ColorRGBA
import time
import numpy as np
from numpy.linalg import norm
import matplotlib.pyplot as plt
import matplotlib.markers as markers
from skimage.measure import find_contours
from sklearn.cluster import AgglomerativeClustering
import random
import threading
import sys
import signal

map_data = OccupancyGrid()
odom = Odometry
bot_x = 0
bot_y = 0
resolution = 0
ori_x = 0
ori_y = 0
target = (0, 0)
prev_target = (0, 0)
rviz_id = 0

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

def bot_index(x, y):
    return np.array([int((x - ori_x)/resolution), int((y - ori_y)/resolution)])

def to_map_coord(contour):
    for index in range(len(contour)):
        contour[index][0] = round(contour[index][0] * resolution + ori_x, 2)
        contour[index][1] = round(contour[index][1] * resolution + ori_y, 2)
    return contour

def get_frontier():
    fig, ax = plt.subplots()
    while map_data.header.seq<1 or len(map_data.data)<1:
        pass
    data = map_data.data
    # print(bot_x, bot_y)
    w = map_data.info.width
    h = map_data.info.height
    data_np = np.asarray(data)
    data_np = np.reshape(data_np, [h, w]);
    bot_idx = bot_index(bot_x, bot_y)
    ## for visualizing ####
    plt.imshow(data_np, cmap='gray', vmin = -1, vmax = 100)
    plt.plot(bot_idx[0], bot_idx[1], 'bo--', linewidth=2, markersize=12)
    contour_m1 = find_contours(data_np, -1, fully_connected='high')
    # edges
    contour_0 = find_contours(data_np, 1, fully_connected='high')
    # for contour in contour_0:
    #     ax.plot(contour[:, 1], contour[:, 0], linewidth=2)
    contour_m1 = np.concatenate(contour_m1, axis=0)
    contour_0 = np.concatenate(contour_0, axis=0)
    
    contour_0 = to_map_coord(contour_0)
    contour_m1 = to_map_coord(contour_m1)

    contour_0 = np.asarray(contour_0)
    contour_m1 = np.asarray(contour_m1)

    # convert contour np arrays into sets
    set_0 = set([tuple(x) for x in contour_0])
    set_m1 = set([tuple(x) for x in contour_m1])

    candidates = list(set_m1.difference(set_0))
    #### uncomment for visualization
    # for can in candidates:
    #     can = np.asarray(can)
    #     ax.plot((can[1]-ori_x)/resolution, (can[0]-ori_y)/resolution, 'rx', linewidth=1, markersize=1)
    candidates = np.asarray(candidates)
    model = AgglomerativeClustering(n_clusters=3, affinity='euclidean', linkage='ward')
    model.fit(candidates)
    labels = model.labels_
    plt.scatter((candidates[labels==0, 1]-ori_x)/resolution, (candidates[labels==0, 0]-ori_y)/resolution, s=50, marker='o', color='red')
    plt.scatter((candidates[labels==1, 1]-ori_x)/resolution, (candidates[labels==1, 0]-ori_y)/resolution, s=50, marker='o', color='white')
    plt.scatter((candidates[labels==2, 1]-ori_x)/resolution, (candidates[labels==2, 0]-ori_y)/resolution, s=50, marker='o', color='green')


    plt.show()


def get_frontier_thread(name, lock):
    print(f'Thread {name} is starting')
    markerarray_publisher = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=10)
    global rviz_id
    rviz_id = 0

    while True:
        while map_data.header.seq<1 or len(map_data.data)<1:
            pass
        data = map_data.data
        # print(bot_x, bot_y)
        w = map_data.info.width
        h = map_data.info.height
        data_np = np.asarray(data)
        data_np = np.reshape(data_np, [h, w]);
        lock.acquire()
        # bot_idx = bot_index(bot_x, bot_y)
        ## for visualizing ####
        # for i in range(0, data_np.shape[0]):
        #     for j in range(0, data_np.shape[1]):
        #         if data_np[i,j] == -1:
        #             data_np[i,j] = 255
        #         elif data_np[i,j] == 100:
        #             data_np[i,j] = 0
        #         elif data_np[i,j] == 0:
        #             data_np[i,j] = 127
        # ax.imshow(data_np, cmap='gray', vmin = -1, vmax = 100)
        # ax.plot(bot_idx[0], bot_idx[1], 'bo--', linewidth=2, markersize=12)
        # unknowns
        contour_m1 = find_contours(data_np, -1, fully_connected='high')
        # edges
        contour_0 = find_contours(data_np, 1, fully_connected='high')
        # for contour in contour_0:
        #     ax.plot(contour[:, 1], contour[:, 0], linewidth=2)
        contour_m1 = np.concatenate(contour_m1, axis=0)
        contour_0 = np.concatenate(contour_0, axis=0)
        
        contour_0 = to_map_coord(contour_0)
        contour_m1 = to_map_coord(contour_m1)

        contour_0 = np.asarray(contour_0)
        contour_m1 = np.asarray(contour_m1)

        # convert contour np arrays into sets
        set_0 = set([tuple(x) for x in contour_0])
        set_m1 = set([tuple(x) for x in contour_m1])

        candidates = np.asarray(set_m1.difference(set_0))
        global target
        target =  random.choice(candidates)
        # print(f'chosen {target}')
        lock.release()

        dim = resolution*2
        markerArray = MarkerArray()
        # loop throgh all instances of the array
        for index in range(len(candidates)):
            marker = Marker()
            marker.id = rviz_id
            marker.header.frame_id = "map"
            marker.type = marker.SPHERE
            marker.action = marker.ADD
            marker.lifetime = rospy.Duration(120.0)
            marker.scale.x = dim 
            marker.scale.y = dim
            marker.scale.z = dim
            marker.color = ColorRGBA(0.0, 0.0, 1.0, 0.75)
            # x and y are inverted due to nature of the map
            marker.pose.position.x = candidates[index, 1]
            marker.pose.position.y = candidates[index, 0]
            markerArray.markers.append(marker)

            rviz_id += 1
            if rviz_id == 1000:
                rviz_id = 0
        markerarray_publisher.publish(markerArray)



def movebase_thread(client, lock):
    # # Create an action client "move_base"
    # client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    global rviz_id
    marker_publisher = rospy.Publisher('target', Marker, queue_size=10)
    marker_goal = Marker()
    marker_goal.id = rviz_id
    marker_goal.header.frame_id = "map"
    marker_goal.type = marker_goal.SPHERE
    marker_goal.action = marker_goal.ADD
    marker_goal.scale.x = 0.5
    marker_goal.scale.y = 0.5
    marker_goal.scale.z = 0.5
    marker_goal.color.a = 1.0
    rviz_id += 1
    if rviz_id == 1000:
        rviz_id = 0
    print(f'Movebase thread starting')
    client.wait_for_server()
    print('connected to server')
    while True:

        lock.acquire()
        global prev_target

        # Sends the goal to the action server.
        if target != prev_target:
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position.x = float(target[1])
            goal.target_pose.pose.position.y = float(target[0]) 
            goal.target_pose.pose.orientation.w = 1.0
            print(f'new target {target}')
            # No rotation of the mobile base frame w.r.t. map frame
            client.send_goal(goal)
            prev_target = target

            marker_goal.pose.orientation.w = 1.0
            marker_goal.pose.position.x = target[1]
            marker_goal.pose.position.y = target[0]
            marker_goal.pose.position.z = 0
            marker_publisher.publish(marker_goal)

        lock.release()
        while norm(np.array([bot_x, bot_y]) - np.array([prev_target[1], prev_target[0]]) > 0.3):
            pass
        # client.wait_for_result()
    # Waits for the server to finish performing the action.
    # If the result doesn't arrive, assume the Server is not available
    # if not client.wait_for_result:
    #     rospy.logerr("Action server not available!")
    #     rospy.signal_shutdown("Action server not available!")
    # else:
    # # Result of executing the action
    #     return client.get_result()   


def signal_handler(sig, frame):
    print('Program ended by user')
    sys.exit()

# If the python node is executed as main process (sourced directly)
if __name__ == '__main__':
    #Initializes a rospy node to let the SimpleActionClient publish and subscribe
    rospy.init_node('movebase_client_py')
    rate = rospy.Rate(10)
    # Create an action client "move_base"
    movebase_client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    odom_subscriber = rospy.Subscriber('/odom', Odometry, update_pose)
    map_client = rospy.Subscriber("/map", OccupancyGrid, map_callback)
    vel_msg = Twist()
    # x_goal = 0
    # y_goal = 0
    # result = movebase_client(x_goal, y_goal)
    # if result:
    #     rospy.loginfo("Goal execution done!")
    lock = threading.Lock()

    get_frontier()

    # threadMap = threading.Thread(target=get_frontier_thread, args=('random frontier', lock, ))
    # threadMap.daemon = True
    # threadMap.start()

    # threadMove = threading.Thread(target=movebase_thread, args=(movebase_client, lock, ))
    # threadMove.daemon = True
    # threadMove.start()
    
    # while threadMap.is_alive():
    #     signal.signal(signal.SIGINT, signal_handler)