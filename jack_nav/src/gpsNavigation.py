#!/usr/bin/env python

import rospy
import time
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

#from gps_to_xyz import get_pose_based_on_lat_long

# initializes the action client node
rospy.init_node('move_base_gps_node')

action_server_name = '/move_base'
client = actionlib.SimpleActionClient(action_server_name, MoveBaseAction)

# waits until the action server is up and running
rospy.loginfo('Waiting for action Server '+action_server_name)
client.wait_for_server()
rospy.loginfo('Action Server Found...'+action_server_name)

# define the list of GPS coordinates to visit
gps_coords = [
    {'lat': 27.7276231, 'long': 68.8177448},
    {'lat': 27.7276720667, 'long': 68.8179089167},
    {'lat': 27.7276720670, 'long': 68.8179089180},
]

def get_xy_based_on_lat_long(lat,lon, name):
    # Define a local orgin, latitude and longitude in decimal degrees
    # GPS Origin
    olat = 49.9
    olon = 8.9
    
    xg2, yg2 = gc.ll2xy(lat,lon,olat,olon)
    utmy, utmx, utmzone = gc.LLtoUTM(lat,lon)
    xa,ya = axy.ll2xy(lat,lon,olat,olon)

   

    return xg2, yg2

# move the robot to each GPS position in the list
for coords in gps_coords:
    x, y, yaw = get_pose_based_on_lat_long(coords['lat'], coords['long'])
    
    # creates a goal to send to the action server
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.z = yaw[2]
    goal.target_pose.pose.orientation.w = yaw[3]

    client.send_goal(goal)
    rospy.loginfo("Sending goal to move to GPS coordinates: {},{}".format(coords['lat'], coords['long']))

    # wait for the robot to reach the goal
    while True:
        state = client.get_state()
        if state == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("Robot has reached the goal!")
            break
        elif state == actionlib.GoalStatus.PREEMPTED:
            rospy.loginfo("Goal was preempted!")
            break
        elif state == actionlib.GoalStatus.ABORTED:
            rospy.loginfo("Goal was aborted!")
            break
        else:
            rospy.loginfo("Robot is still moving towards the goal...")
            rospy.sleep(1.0)

rospy.loginfo("Robot has reached all the goals. Mission accomplished!") 

