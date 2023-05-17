from utm import from_latlon
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Twist
from tf.transformations import euler_from_quaternion
import math


class JackalNavigator:
    def __init__(self, current_lat, current_lon, dest_lat, dest_lon):
        self.current_pos = None
        self.dest_pos = Point()
        x, y, zone_number, zone_letter = from_latlon(dest_lat, dest_lon)
        self.dest_pos.x = x
        self.dest_pos.y = y
        self.dest_pos.z = 0
        self.current_bearing =0
        self.odom_sub = rospy.Subscriber('/jackal_velocity_controller/odom', Odometry, self.odom_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.set_initial_position(current_lat, current_lon)

    def set_initial_position(self, lat, lon):
        x, y, zone_number, zone_letter = from_latlon(lat, lon)
        self.current_pos = Point()
        self.current_pos.x = x
        self.current_pos.y = y
        self.current_pos.z = 0
        print(x)
        print(y)

    def run(self):
        rate = rospy.Rate(10)  # run loop at 10 Hz
        print("Starting ............")
        while not rospy.is_shutdown():
            if self.current_pos is not None:
                # calculate bearing angle to destination
                dest_bearing = math.degrees(math.atan2(self.dest_pos.y - self.current_pos.y, self.dest_pos.x - self.current_pos.x))
                print("Destination Bearing",dest_bearing)
                if dest_bearing < 0:
                    dest_bearing += 360

                # calculate difference between current and destination bearings
                bearing_diff = dest_bearing - self.current_bearing
            
                print("Current Bearing",self.current_bearing)
                print("Beaing Difference",bearing_diff)
                if bearing_diff < -180:
                    bearing_diff += 360
                elif bearing_diff > 180:
                    bearing_diff -= 360

                # set forward and angular velocities based on bearing difference
                twist = Twist()
                if abs(bearing_diff) < 10:
                    twist.linear.x = 0.5
                    twist.angular.z = 0
                elif bearing_diff > 0:
                    twist.linear.x = 0
                    twist.angular.z = 0.5
                else:
                    twist.linear.x = 0
                    twist.angular.z = -0.5
                self.cmd_vel_pub.publish(twist)

                # print current position and bearing angle
                rospy.loginfo("Current position: x={current_pos.x}, y={current_pos.y}, bearing={current_bearing}")
                

            
            rate.sleep()

    def odom_callback(self, msg):
        # get current position from odometry message
        self.current_pos = msg.pose.pose.position

        # get current orientation from odometry message and convert to bearing angle
        quat = msg.pose.pose.orientation
        (roll, pitch, yaw) = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        self.current_bearing = math.degrees(yaw)

        # adjust bearing angle to be in the range [0, 360)
        if self.current_bearing < 0:
            self.current_bearing += 360

if __name__ == '__main__':
    rospy.init_node('jackal_navigator')
    navigator = JackalNavigator(27.7276231, 68.8177448, 27.7276720667, 68.8179089167)
    navigator.run()
