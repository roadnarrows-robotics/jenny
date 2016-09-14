#!/usr/bin/env python

import rospy
import gps

from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import NavSatStatus

def read_gps():
    pub = rospy.Publisher('gpsRoverStatus', NavSatFix, queue_size=10)
    rospy.init_node('jenny_gps', anonymous=True)
    rate = rospy.Rate(20)
    session = gps.gps("localhost", "2947")
    session.stream(gps.WATCH_ENABLE | gps.WATCH_NEWSTYLE)
    navMsg = NavSatFix()
    navMsg.header.frame_id = 'gpsRover'
    navStatus = NavSatStatus()
    navStatus.service  = NavSatStatus.SERVICE_GPS
    while not rospy.is_shutdown():
        try:
            navMsg.latitude = session.fix.latitude
            navMsg.longitude = session.fix.longitude
            if(session.status > 0):
                navStatus.status = NavSatStatus.STATUS_FIX
            else:
                navStatus.status = NavSatStatus.STATUS_NO_FIX
            navMsg.status = navStatus
            navMsg.header.stamp = rospy.Time.now()
            rospy.loginfo(navMsg)
            pub.publish(navMsg)
            rate.sleep()
        except AttributeError as e:
            rospy.logwarn("Attrbute Error in GPS Data %s" % e)
        except KeyError:
            rospy.logwarn("Key Error in GPS Data")

if __name__ == '__main__':
    try:
        read_gps()
    except rospy.ROSInterruptException:
        pass 
