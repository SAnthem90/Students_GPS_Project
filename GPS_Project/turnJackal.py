import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

# Global variables for the Jackal's position and orientation
x = 0.0
y = 0.0
yaw = 0.0

def odom_callback(msg):
    global x, y, yaw
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    orientation = msg.pose.pose.orientation
    (roll, pitch, yaw) = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
    

def turn_jackal():
    # Initialize the ROS node
    rospy.init_node('turn_jackal', anonymous=True)

    # Create a publisher to the "cmd_vel" topic
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    # Subscribe to the "odom" topic to get the Jackal's pose
    odom_sub = rospy.Subscriber('/jackal_velocity_controller/odom', Odometry, odom_callback)
    print(yaw)
    


    # Wait for the first Odometry message to arrive
    while not rospy.is_shutdown() and odom_sub.get_num_connections() == 0:
        rospy.loginfo("Waiting for /odom topic...")
        rospy.sleep(1)

    if rospy.is_shutdown():
        return

    rospy.loginfo("Received /odom topic")

    # Create a Twist message and set its angular z component to 0.5 rad/s
    twist_msg = Twist()
    twist_msg.angular.z = 0.5

    # Turn the Jackal 90 degrees
    start_yaw = yaw
    while not rospy.is_shutdown() and abs(yaw - start_yaw) < 1.57: # 90 degrees in radians
        cmd_vel_pub.publish(twist_msg)

    # Stop the Jackal after it has turned 90
    twist_msg.angular.z = 0
    cmd_vel_pub.publish(twist_msg)

if __name__ == '__main__':
    try:
        turn_jackal()
    except rospy.ROSInterruptException:
        pass
