#!/usr/bin/env python

#--------Include modules---------------
from copy import copy
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
import tf
from auto_slam.msg import PointArray
from time import time
from numpy import array
from numpy import linalg as LA
from numpy import all as All
from numpy import inf
from functions import robot,informationGain
from numpy.linalg import norm

# Subscribers' callbacks------------------------------
mapData=OccupancyGrid()
frontiers=[]
def callBack(data):
	global frontiers
	frontiers=[]
	for point in data.points:
		frontiers.append(array([point.x,point.y]))

def mapCallBack(data):
    global mapData
    mapData=data
# Node----------------------------------------------

def node():
	global frontiers,mapData
	rospy.init_node('assigner', anonymous=False)
	
	# fetching all parameters
	map_topic= rospy.get_param('~map_topic','/map')
	info_radius= rospy.get_param('~info_radius',1.0)					#this can be smaller than the laser scanner range, >> smaller >>less computation time>> too small is not good, info gain won't be accurate
	info_multiplier=rospy.get_param('~info_multiplier',3.0)		
	hysteresis_radius=rospy.get_param('~hysteresis_radius',3.0)			#at least as much as the laser scanner range
	hysteresis_gain=rospy.get_param('~hysteresis_gain',2.0)				#bigger than 1 (biase robots to continue exploring current region
	frontiers_topic= rospy.get_param('~frontiers_topic','/filtered_points')	
	delay_after_assignement=rospy.get_param('~delay_after_assignement',0.5)
	rateHz = rospy.get_param('~rate',100)
	rate = rospy.Rate(rateHz)
#-------------------------------------------
	rospy.Subscriber(map_topic, OccupancyGrid, mapCallBack)
	rospy.Subscriber(frontiers_topic, PointArray, callBack)
#---------------------------------------------------------------------------------------------------------------
	pub = rospy.Publisher('RRT_goal', Marker, queue_size=10)
#--------------------------------------------------------------------------------------------------------------- 
# wait if no frontier is received yet 
	while len(frontiers)<1:
		pass
	centroids=copy(frontiers)

#Set the marker action for assigned goal. 
	p=Point()
	points_RRT=Marker()
	points_RRT.header.frame_id= mapData.header.frame_id
	points_RRT.header.stamp= rospy.Time.now()
	points_RRT.ns= "markers1"
	points_RRT.id = 1
	points_RRT.type = Marker.POINTS
	points_RRT.action = Marker.ADD
	points_RRT.pose.orientation.w = 1.0
	points_RRT.scale.x=0.2
	points_RRT.scale.y=0.2 
	points_RRT.color.r = 0.0/255.0
	points_RRT.color.g = 255.0/255.0
	points_RRT.color.b = 255.0/255.0
	points_RRT.color.a=1
	points_RRT.lifetime = rospy.Duration()

# wait if map is not received yet
	while (len(mapData.data)<1):
		pass
	robots = robot()
#-------------------------------------------------------------------------
#---------------------     Main   Loop     -------------------------------
#-------------------------------------------------------------------------
	while not rospy.is_shutdown():
		centroids=copy(frontiers)
#-------------------------------------------------------------------------			
#Get information gain for each frontier point
		infoGain=[]
		for ip in range(0,len(centroids)):
			infoGain.append(informationGain(mapData,[centroids[ip][0],centroids[ip][1]],info_radius))
#-------------------------------------------------------------------------				
		revenue_record=[]
		centroid_record=[]
		id_record=[]
		ir=[]
		
		for ip in range(0,len(centroids)):
			cost=norm(robots.getPosition()-centroids[ip])	
			information_gain=infoGain[ip]
			if (norm(robots.getPosition()-centroids[ip])<=hysteresis_radius):
				information_gain*=hysteresis_gain
			revenue=information_gain*info_multiplier-cost
			revenue_record.append(revenue)
			centroid_record.append(centroids[ip])
			id_record.append(ir)
#-------------------------------------------------------------------------	
		if (len(id_record)>0):
			winner_id=revenue_record.index(max(revenue_record))
			p.x=centroid_record[winner_id][0]
			p.y=centroid_record[winner_id][1]
			rrt_goal=[]
			rrt_goal.append(copy(p))
			points_RRT.points=rrt_goal
			pub.publish(points_RRT)	
			robots.sendGoal(centroid_record[winner_id])
			rospy.sleep(delay_after_assignement)
#------------------------------------------------------------------------- 
		rate.sleep()
#-------------------------------------------------------------------------

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass