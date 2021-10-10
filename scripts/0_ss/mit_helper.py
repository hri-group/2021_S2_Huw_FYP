#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose, PoseStamped, Twist, Vector3,Vector3Stamped,PoseWithCovariance,TwistWithCovariance,Point
from nav_msgs.msg import Odometry
from ford_msgs.msg import PedTrajVec,PedTraj,Pose2DStamped,Clusters
import tf

from pedsim_msgs.msg import *

from copy import deepcopy



class mit_helper():
    def __init__(self):
        self.ped_info = []

        # tf listener
        self.listener = tf.TransformListener()

        # converting odom to pose and velocity messages
        self.odom_converter = rospy.Subscriber('~odom', Odometry, self.cbOdom)
        self.pose_publisher = rospy.Publisher('~pose',PoseStamped,queue_size=1) # position in global frame
        self.vel_publisher = rospy.Publisher('~velocity',Vector3,queue_size=1) # velocity in global frame
        

        # converting pedsim info to pedtraj messages
        self.sub_peds = rospy.Subscriber('/pedsim_simulator/simulated_agents', AgentStates, self.cbPeds)
        self.ped_traj_publisher = rospy.Publisher('~pedtraj',Clusters,queue_size=1)

    def cbPeds(self,msg):
        ped_location = Clusters()
        for i in xrange(len(msg.agent_states)):

            ped_location.labels.append(msg.agent_states[i].id)
            ped_location.mean_points.append(msg.agent_states[i].pose.position)
            ped_location.velocities.append(msg.agent_states[i].twist.linear)

        self.ped_info = deepcopy(ped_location)

        self.ped_traj_publisher.publish(self.ped_info)


    def cbOdom(self,msg):
        # pose 
        pose = PoseStamped()
        pose.pose = msg.pose.pose
        pose.header = msg.header

        # velocity
        local_vel = Vector3Stamped()
        local_vel.vector = msg.twist.twist.linear
        local_vel.header = deepcopy(msg.header)
        local_vel.header.frame_id = deepcopy(msg.child_frame_id)
        try:
            global_vel = self.listener.transformVector3(msg.header.frame_id,local_vel)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return

        self.pose_publisher.publish(pose)
        self.vel_publisher.publish(global_vel.vector)
        
            

if __name__ == '__main__':
    rospy.init_node('mit_helper', anonymous=False)
    print "Topic converter online."
    helper = mit_helper()
    rospy.spin()
