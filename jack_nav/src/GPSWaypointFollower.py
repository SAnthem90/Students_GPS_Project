#!/usr/bin/env python

import rospy
import tf2_ros
from geographic_msgs.msg import GeoPoint
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib

class GPSWaypointFollower:
    def __init__(self):
        # Initialize the node
        rospy.init_node('gps_waypoint_follower')

        # Set up the TF2 buffer and listener
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)

        # Set up the publisher for the planned path
        self.pathPub = rospy.Publisher('/planned_path', Path, queue_size=10)

        # Set up the client for the move_base action server
        self.moveBaseClient = actionlib.SimpleActionClient('move_base', MoveBaseAction)

        # Set the rate of the node
        self.rate = rospy.Rate(10)

    def run(self):
        # Define the GPS waypoints
        waypoints = [
            GeoPoint(latitude=YOUR_LATITUDE_1, longitude=YOUR_LONGITUDE_1),
            GeoPoint(latitude=YOUR_LATITUDE_2, longitude=YOUR_LONGITUDE_2),
            
            GeoPoint(latitude=YOUR_LATITUDE_N, longitude=YOUR_LONGITUDE_N)
        ]

        # Convert the GPS coordinates to local coordinates in the map frame
        localWaypoints = []
        for waypoint in waypoints:
            # Convert the GPS coordinate to a PoseStamped in the map frame
            mapPose = PoseStamped()
            mapPose.header.frame_id = 'map'
            mapPose.pose.position.x = 0
            mapPose.pose.position.y = 0
            mapPose.pose.position.z = 0
            mapPose.pose.orientation.x = 0
            mapPose.pose.orientation.y = 0
            mapPose.pose.orientation.z = 0
            mapPose.pose.orientation.w = 1
            mapPose.header.stamp = rospy.Time.now()
            mapPose = self.tfBuffer.transform(mapPose, 'gps', rospy.Duration(1.0))

            # Convert the PoseStamped to a GeoPoint
            mapGeo = GeoPoint()
            mapGeo.latitude = mapPose.pose.position.x
            mapGeo.longitude = mapPose.pose.position.y

            # Append the local coordinates to the list of waypoints
            localWaypoints.append(mapGeo)

        # Generate a sequence of goals for the robot to follow
        goals = []
        for waypoint in localWaypoints:
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = 'map'
            goal.target_pose.pose.position.x = waypoint.latitude
            goal.target_pose.pose.position.y = waypoint.longitude
            goal.target_pose.pose.position.z = 0
            goal.target_pose.pose.orientation.x = 0
            goal.target_pose.pose.orientation.y = 0
            goal.target_pose.pose.orientation.z = 0
            goal.target_pose.pose.orientation.w = 1
            goals.append(goal)

        # Send the goals to the move_base action server
        rospy.loginfo("Sending goals to move_base server")

        for goal in goals:
            
            print("Sending goal:", goal)
            # Create a goal message
            move_goal = MoveBaseGoal()

            # Set the frame ID and timestamp
            move_goal.target_pose.header.frame_id = 'map'
            move_goal.target_pose.header.stamp = rospy.Time.now()

            # Set the goal position
            move_goal.target_pose.pose.position.x = goal[0]
            move_goal.target_pose.pose.position.y = goal[1]
            move_goal.target_pose.pose.position.z = 0.0

            # Set the goal orientation
            move_goal.target_pose.pose.orientation.x = 0.0
            move_goal.target_pose.pose.orientation.y = 0.0
            move_goal.target_pose.pose.orientation.z = goal[2]
            move_goal.target_pose.pose.orientation.w = goal[3]

            # Send the goal to the move_base action server
            move_base_client.send_goal(move_goal)

            # Wait for the robot to reach the goal
            move_base_client.wait_for_result()

            # Check if the goal was reached
            if move_base_client.get_state() == GoalStatus.SUCCEEDED:
                rospy.loginfo("Goal reached")
            else:
                rospy.loginfo("Failed to reach goal")
