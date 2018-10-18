#!/usr/bin/python
import rospy
from nav_msgs.msg import Path
from tf.transformations import euler_from_quaternion
#from geometry_msgs.msg import Point, Twist
from bringup_dual.msg import commendMsg

class cmd_publisher:

    def __init__(self):
        rospy.init_node('cmd_publisher', anonymous=True)
        self.path_topic = rospy.get_param('~path_topic', '/move_base/TebLocalPlannerROS/local_plan')
        self.pub = rospy.Publisher('/ns1/cmd_msg', commendMsg, queue_size=100)
        self.sub = rospy.Subscriber(self.path_topic, Path, self.pathCallback)
        self.pose_cmd = commendMsg()

    def pathCallback(self, data):
        if len(data.poses)>10:
            pose = data.poses[10].pose
        else:
            pose = data.poses[-1].pose
        self.pose_cmd.xd = pose.position.x
        self.pose_cmd.yd = pose.position.y
        self.pose_cmd.phid = euler_from_quaternion([
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w])[2]
        self.pub.publish(self.pose_cmd)

    def node(self):
        rospy.spin()


if __name__=='__main__':
    cmd_pub=cmd_publisher()
    try:
        cmd_pub.node()
    except rospy.ROSInterruptException: pass

