#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

def callbackScan(data):
	global isObstacleFront, isObstacleBack
	rospy.loginfo('================= in callbackScan =============')

	pointCountFront = 0
	pointCountBack = 0
	# define rectangular region in front and back of robot to check for obstacle
	# x points left of robot, y points back of robot
	y_min_front = -1.5
	y_max_front = 0 # how close an obstacle in front has to be before the robot stops when going forward
	x_min_front = -0.25
	x_max_front = 0.25

	y_min_back = 0 # how close an obstacle behind has to be before the robot stops going backwards
	y_max_back = 2.0
	x_min_back = -0.25
	x_max_back = 0.25

	pointsThres = 2 # min number of pointcloud points to be considered as obstacle and not noise

	theta = data.angle_min

	rospy.loginfo(data.ranges[0])

	for pointDist in data.ranges:
		# zero degree points left of robot 
		x_coord = pointDist * math.cos(theta)
		y_coord = pointDist * math.sin(theta)
		if (x_coord > x_min_front) and (x_coord < x_max_front) and (y_coord > y_min_front) and (y_coord < y_max_front):
			pointCountFront=pointCountFront + 1 
		elif (x_coord > x_min_back) and (x_coord < x_max_back) and (y_coord > y_min_back) and (y_coord < y_max_back):
			pointCountBack=pointCountBack+1
		theta = theta + data.angle_increment

	rospy.loginfo('pointCountFront %f', pointCountFront)
	rospy.loginfo('pointCountBack %f', pointCountBack)

	if pointCountFront > pointsThres:
		isObstacleFront = True 
		rospy.loginfo('isObstacleFront true')
	else:
		isObstacleFront = False
		rospy.loginfo('isObstacleFront false')

	if pointCountBack > pointsThres:
		isObstacleBack = True
		rospy.loginfo('isObstacleBack true')
	else:
		isObstacleBack = False
		rospy.loginfo('isObstacleBack false')
		
def callbackTwist(msg):
	global isObstacleFront, isObstacleBack
	
	if msg.linear.x > 0 and isObstacleFront:
		msg.linear.x = 0
	elif msg.linear.x < 0 and isObstacleBack:
		msg.linear.x = 0
	twist_publisher.publish(msg)


if __name__ == '__main__':
	isObstacleFront = False
	isObstacleBack = False
	rospy.init_node('collision_avoidance',anonymous=True)

	rospy.Subscriber("/scan", LaserScan, callbackScan)
	rospy.Subscriber("/mit_cadrl/cmd_vel", Twist, callbackTwist)
	twist_publisher = rospy.Publisher('/cmd_vel',Twist,queue_size=1) # position in global frame

	rate = rospy.Rate(100) #100Hz
	while not rospy.is_shutdown():
		rate.sleep()
	rospy.loginfo('node shutdown')
