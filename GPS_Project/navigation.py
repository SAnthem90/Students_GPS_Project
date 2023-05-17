#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

def move_jackal():
    # Initialize the ROS node
    rospy.init_node('move_jackal', anonymous=True)

    # Create a publisher to the "cmd_vel" topic
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    # Create a Twist message and set its linear x component to 0.2 m/s
    twist_msg = Twist()
    twist_msg.linear.x = 0.2

    # Move the Jackal 2 meters straight
    distance_moved = 0
    start_time = rospy.get_time()

    rate = rospy.Rate(10) # 10 Hz

    while not rospy.is_shutdown() and distance_moved < 10:
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


if __name__ == '__main__':
    try:
        move_jackal()
    except rospy.ROSInterruptException:
        pass
