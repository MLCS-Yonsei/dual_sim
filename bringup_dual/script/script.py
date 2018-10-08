#!/usr/bin/env python

#--------Include modules---------------
import rospy
from bringup_dual.msg import commendMsg

def node():
    q=commendMsg()
    pub = rospy.Publisher('/ns1/cmd_msg', commendMsg, queue_size=100)
    rospy.init_node('script', anonymous=True)
    q.xd=10
    q.yd=10
    q.phid=1

    while not rospy.is_shutdown():
        pub.publish(q)




if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException: pass