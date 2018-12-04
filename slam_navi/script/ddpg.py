#!/usr/bin/env python

#--------Include modules---------------
import os
import tf
from copy import copy
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist
from time import time
import numpy as np
from actor_net import ActorNet

# Subscribers' callbacks----------------------------------------
scandata=[]
path=[]
goal_from_path = 15
    
def scanCallBack(data):
	global scandata
	scandata=[]
	scandata_raw=np.array(data.ranges)
	scandata_raw[scandata_raw==np.inf] = 10.0
	for idx in range(36):
		scandata.append(min(scandata_raw[idx*10:idx*10+10])/10.0)

def pathCallback(data):
	global path
	path=[]
	for pose in data.poses:
		path.append(list([pose.pose.position.x, pose.pose.position.y, tf.transformations.euler_from_quaternion([
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w])[2]]))
	
# Node------------------------------------------------------------------

def node():
	global scandata
	rospy.init_node('ddpg', anonymous=False)
	agent = ActorNet(num_states=36*4 + 2 + 2, num_actions=2)

	# load weights
	agent.load_actor(os.path.abspath(__file__).replace('ddpg.py','weights/actor/model.ckpt'))
	scan_topic = rospy.get_param('~scan_topic','/scan')
	path_topic = rospy.get_param('~path_topic','/move_base/TebLocalPlannerROS/local_plan')
	rate = rospy.Rate(rospy.get_param('~rate',10))
#-------------------------------------------------------------------------
	rospy.Subscriber(scan_topic, LaserScan, scanCallBack)
	rospy.Subscriber(path_topic, Path, pathCallback)
#-------------------------------------------------------------------------
	pub = rospy.Publisher('ddpg_goal', Marker, queue_size=10) 
	pub2 = rospy.Publisher('cmd_vel', Twist, queue_size=10) 
#-------------------------------------------------------------------------
	listener = tf.TransformListener()
		
	q=Point()
	
	points_ddpg=Marker()
	points_ddpg.header.frame_id= "map"
	points_ddpg.header.stamp= rospy.Time.now()
	points_ddpg.ns= "markers1"
	points_ddpg.id = 1
	points_ddpg.type = Marker.POINTS
	points_ddpg.action = Marker.ADD
	points_ddpg.pose.orientation.w = 1.0
	points_ddpg.scale.x=0.5
	points_ddpg.scale.y=0.5 
	points_ddpg.color.r = 0.0/255.0
	points_ddpg.color.g = 255.0/255.0
	points_ddpg.color.b = 255.0/255.0
	points_ddpg.color.a=1
	points_ddpg.lifetime = rospy.Duration()
		
#-------------------------------------------------------------------------
#---------------------     Main   Loop     -------------------------------
#-------------------------------------------------------------------------
	
	print("Main Loop runnig...")
	trans=[]
	rot=[]
	vel=Twist()
	prev_action = [0.0, 0.0]
	scanbuffer = None
	goal_a = [0, -2]
	while not rospy.is_shutdown():
		listener.waitForTransform('/map', '/base_link', rospy.Time(0), rospy.Duration(10.0))
		cond=0
		# print("scandata : ", scandata)
		while cond==0:	
			try:
				(trans,rot) = listener.lookupTransform('/map', '/base_link', rospy.Time.now())
				cond=1
			except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
				cond==0
			
			position = trans[0:2]
			orientation = [list(tf.transformations.euler_from_quaternion(rot))[2]]
			pose_world = position + orientation
			# print("path length : " , len(path))
		q.x=goal_a[0]
		q.y=goal_a[1]
		goal=[]
		qq=[]
		qq.append(copy(q))
		points_ddpg.points=qq
		pub.publish(points_ddpg)
		
		
		goal.append(goal_a[0]- pose_world[0])
		goal.append(goal_a[1]- pose_world[1])

		goal[0:2] = [
			(np.sin(pose_world[2])*goal[0]-np.cos(pose_world[2])*goal[1])/5.0,
			(np.cos(pose_world[2])*goal[0]+np.sin(pose_world[2])*goal[1])/5.0
		]
		if goal[0]**2+goal[1]**2>0.01:
			if type(scanbuffer)==type(None):
				scanbuffer = scandata*4
			else:
				scanbuffer[108:144] = scandata
			state = np.array(scanbuffer + prev_action + goal).reshape([1,-1])
			action = agent.evaluate_actor(state).reshape([-1])
			print('if 1 Current Decision:',action)
			vel.linear.x = -0.4*action[0]
			vel.linear.y = -0.4*action[1]
			vel.linear.z = 0
			vel.angular.x = 0
			vel.angular.y = 0
			vel.angular.z = 0
			pub2.publish(vel)
			prev_action = list(action.reshape([-1]))
			scanbuffer[0:108] = scanbuffer[36:144]
		else:
			vel.linear.x = 0
			vel.linear.y = 0
			vel.linear.z = 0
			vel.angular.x = 0
			vel.angular.y = 0
			vel.angular.z = 0
			pub2.publish(vel)


#------------------------------------------------------------------------- 
		rate.sleep()
#-------------------------------------------------------------------------

if __name__ == '__main__':
	try:
		node()
	except rospy.ROSInterruptException:
		pass
 
 
 
 
