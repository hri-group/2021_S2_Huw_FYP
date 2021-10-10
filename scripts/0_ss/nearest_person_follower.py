#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose, PoseStamped, Twist, Vector3,Vector3Stamped,PoseWithCovariance,TwistWithCovariance,Point
from spencer_tracking_msgs.msg import TrackedPerson, TrackedPersons
from nav_msgs.msg import Odometry
# from ford_msgs.msg import PedTrajVec,PedTraj,Pose2DStamped,Clusters
import tf

# from pedsim_msgs.msg import *
from actionlib_msgs.msg import GoalID
from copy import deepcopy

class nearest_person_follower():
    def __init__(self):
        self.target_pose = []
        self.last_tracked_person_time = rospy.get_time()

        # tf listener
        self.listener = tf.TransformListener()

        # converting spencer people tracking info to pedtraj messages
        self.sub_peds = rospy.Subscriber('/spencer/perception/tracked_persons', TrackedPersons, self.cbPeds)
        self.goal_publisher = rospy.Publisher('/move_base_simple/goal',PoseStamped,queue_size=1)
        self.cancel_goal_publisher = rospy.Publisher('move_base/cancel',GoalID,queue_size=1)

    def cbPeds(self,msg):
        if len(msg.tracks) == 0:
            if rospy.get_time() - self.last_tracked_person_time > 5:
                empty_goal = GoalID()
                self.cancel_goal_publisher.publish(empty_goal)
                self.last_tracked_person_time = rospy.get_time()
                print('Cancelling move base goal')
            return
            
        pose_stamped = PoseStamped()
        pose_stamped.header = msg.header

        pose_stamped.pose = deepcopy(msg.tracks[0].pose.pose)
        in_robot_frame = self.listener.transformPose('base_link',pose_stamped)
        min_dist_from_robot = in_robot_frame.pose.position.x**2 + in_robot_frame.pose.position.y**2 + in_robot_frame.pose.position.z**2
        min_i = 0

        for i in xrange(1,len(msg.tracks)):
            pose_stamped.pose = deepcopy(msg.tracks[i].pose.pose)
            in_robot_frame = self.listener.transformPose('base_link',pose_stamped)

            dist_from_robot = in_robot_frame.pose.position.x**2 + in_robot_frame.pose.position.y**2 + in_robot_frame.pose.position.z**2
            if dist_from_robot < min_dist_from_robot:
                min_dist_from_robot = dist_from_robot
                min_i = i



        target_pose = PoseStamped()
        target_pose.header = msg.header
        target_pose.pose = msg.tracks[min_i].pose.pose

        self.target_pose = deepcopy(target_pose)

        self.goal_publisher.publish(self.target_pose)
        self.last_tracked_person_time = rospy.get_time()
        
            

if __name__ == '__main__':
    rospy.init_node('nearest_person_follower', anonymous=False)
    print "Topic converter online."
    helper = nearest_person_follower()
    rospy.spin()
