#!/usr/bin/python
import rospy
from nav_msgs.msg import Path
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
class cmd_publisher:

    def __init__(self):
        rospy.init_node('cmd_publisher', anonymous=True)
        self.cmd_topic = rospy.get_param('~cmd_topic', '/cmd_vel')
        self.pub = rospy.Publisher('/cmd_vel_real', Twist, queue_size=100)
        self.sub = rospy.Subscriber(self.cmd_topic, Twist, self.cmdCallback)
        self.vel_cmd = Twist()
        self.norm_limit = 0.4

    def cmdCallback(self, data):
        vx = data.linear.x
        vy = data.linear.y
        wz = data.angular.z
        norm = abs(vx)+abs(vy)+abs(wz)
        if norm > self.norm_limit:
            vx *= self.norm_limit/norm
            vy *= self.norm_limit/norm
            wz *= self.norm_limit/norm
        
        self.vel_cmd.linear.x = vx
        self.vel_cmd.linear.y = vy
        self.vel_cmd.angular.z = wz
        self.pub.publish(self.vel_cmd)

    def node(self):
        rospy.spin()


if __name__=='__main__':
    cmd_pub=cmd_publisher()
    try:
        cmd_pub.node()
    except rospy.ROSInterruptException: pass