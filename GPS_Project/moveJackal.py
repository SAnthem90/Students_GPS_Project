#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

def odom_callback(data):
    global x, y, yaw
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    orientation = data.pose.pose.orientation
    (roll, pitch, yaw) = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])

def move_jackal():
    # Initialize the ROS node
    rospy.init_node('move_jackal', anonymous=True)

    # Create publishers to the "cmd_vel" and "reset_odom" topics
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    reset_odom_pub = rospy.Publisher('/reset_odometry', Twist, queue_size=10)

    # Subscribe to the "odom" topic to get the Jackal's pose
    rospy.Subscriber('/jackal_velocity_controller/odom', Odometry, odom_callback)

    # Wait for the first Odometry message to arrive
    rospy.wait_for_message('/jackal_velocity_controller/odom', Odometry)

    # Create a Twist message and set its linear x component to 0.2 m/s
    twist_msg = Twist()
    twist_msg.linear.x = 0.2

    # Move the Jackal 2 meters straight
    start_x = x
    start_y = y
    start_yaw = yaw
    distance_moved = 0
    start_time = rospy.get_time()

    rate = rospy.Rate(10) # 10 Hz

    while not rospy.is_shutdown() and distance_moved < 2:
        # Publish the Twist message
        cmd_vel_pub.publish(twist_msg)

        # Compute the distance moved using the elapsed time
        elapsed_time = rospy.get_time() - start_time
        distance_moved = twist_msg.linear.x * elapsed_time

        # Sleep for the remaining time to maintain the loop rate
        rate.sleep()

    # Stop the Jackal after it has moved 2 meters
    twist_msg.linear.x = 0
    cmd_vel_pub.publish(twist_msg)

    # Reset the odometry
    reset_odom_msg = Twist()
    reset_odom_msg.linear.x = 1
    reset_odom_pub.publish(reset_odom_msg)

    # Wait for the odometry to be reset
    rospy.sleep(1)

    # Turn the Jackal 90 degrees
    twist_msg.angular.z = 0.5 # 0.5 rad/s

    while not rospy.is_shutdown() and abs(yaw - start_yaw) < 1.57: # 90 degrees in radians
        cmd_vel_pub.publish(twist_msg)
        rate.sleep()

    # Stop the Jackal after it has turned 90 degrees
    twist_msg.angular.z = 0
    cmd_vel_pub.publish(twist_msg)

    # Move the Jackal 3 meters straight
    start_x = x
    start_y = y
    distance_moved = 0
    start_time = rospy.get_time()

    while not rospy.is_shutdown() and distance_moved < 3:
        cmd_vel_pub.publish(twist_msg)
        elapsed_time = rospy.get_time() - start_time
        distance_moved = twist_msg.linear.x * elapsed_time
        rate.sleep()

    # Stop the Jackal after it has moved 3 meters
    twist_msg.linear.x = 0
    cmd_vel_pub.publish(twist_msg)

if __name__ == '__main__':
    move_jackal()

