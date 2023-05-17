
#!/usr/bin/env python

import rospy
from sensor_msgs.msg import NavSatFix

class JackalGPS:
    def __init__(self):
        rospy.init_node('jackal_gps', anonymous=True)
        self.file_label = rospy.get_param('~file_label', 'gps_data')
        self.file_path = '/tmp/{}_{}.txt'.format(self.file_label, rospy.Time.now().to_nsec())
        self.gps_sub = rospy.Subscriber('/navsat/fix', NavSatFix, self.gps_callback)

    def gps_callback(self, msg):
        lat = msg.latitude
        lon = msg.longitude
	print("Latitude is ",lat)
	print("Longitude is ",lon)
	print()
        with open(self.file_path, 'a') as f:
            f.write('{},{},{}\n'.format(rospy.Time.now().to_nsec(), lat, lon))

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        jackal_gps = JackalGPS()
        jackal_gps.run()
    except rospy.ROSInterruptException:
        pass

