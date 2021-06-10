#!/usr/bin/env python
import rospy
from pedsim_msgs.msg import *
from geometry_msgs.msg import Pose, PoseStamped, Twist, Vector3,PoseWithCovariance,TwistWithCovariance,Point
from nav_msgs.msg import Odometry
import random
import math
import copy
import numpy as np

def distance_between(point1,point2):
    # finds distance between two point messages. 
    x_sep = point1.x - point2.x
    y_sep = point1.y - point2.y
    z_sep = point1.z - point2.z

    linear_sep = math.sqrt(x_sep**2 + y_sep**2 + z_sep**2)

    return linear_sep
class Policy():

    def __init__(self):
        self.name = "Following policy"

    def predict(self,env):

        return self.follow(env)
    
    def follow(self,env):
        # env = [robot_pos, ped_pos_array] all stored as point messages for now
        robot_pos = env[0]
        ped_pos_array = env[1]

        if len(ped_pos_array) == 0:
            twist = Twist()
            return twist

        # find closest pedestrian
        min_dist = distance_between(robot_pos,ped_pos_array[0])
        min_ped = ped_pos_array[0]
        for i in xrange(1,len(ped_pos_array)):
            if distance_between(robot_pos,ped_pos_array[i]) < min_dist:
                min_dist = distance_between(robot_pos,ped_pos_array[i])
                min_ped = ped_pos_array[i]


        # test to see if we can publish twist with linear. 
        x_vel = min_ped.x - robot_pos.x
        y_vel = min_ped.y - robot_pos.y

        twist = Twist()
        twist.linear.x = x_vel
        twist.angular.z = 0

        return twist

    
    def predict_random(self):
        twist = Twist()
        twist.angular.z = 10*(random.random() - 0.5)
        twist.linear.x = 0.2
        return twist


class Jackal():
    def __init__(self):
        self.node_name = rospy.get_name()

        # predictions 
        self.policy = Policy()
        
        # for subscribers

        # robot info
        self.pose = PoseStamped()
        self.vel = Vector3()
        self.psi = 0.0 # heading angle in odom (I think it's odom?)
        self.odom = Odometry()

        # ped info
        self.ped_info = [] # currently has position info of all pedestrians stored in an array of Point messages (each entry has .x,.y,.z values)
        self.ped_traj_vec = []
        self.other_agents_state = []

        # actions
        self.action = Twist()

        # subscribers and publishers
        self.pub_twist = rospy.Publisher('/cmd_vel',Twist,queue_size=1)
        self.sub_peds = rospy.Subscriber('/pedsim_simulator/simulated_agents', AgentStates, self.cbPeds)
        self.sub_odom = rospy.Subscriber('/odometry/filtered',Odometry,self.cbOdom)
        self.sub_pose = rospy.Subscriber('~pose',Pose,self.cbPose)

        # callback control timer
        self.control_timer = rospy.Timer(rospy.Duration(0.01),self.cbControl) # publishes Twist message
        self.nn_timer = rospy.Timer(rospy.Duration(0.1),self.cbComputeAction) # calculates new action based on neural network. 

    def cbPose(self, msg):
        q = msg.pose.orientation
        self.psi = np.arctan2(2.0*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)) # bounded by [-pi, pi]
        self.pose = msg
        print self.psi

    def cbPeds(self,msg):
        ped_locations = []
        for i in xrange(len(msg.agent_states)):
            ped_locations.append(msg.agent_states[i].pose.position)
        self.ped_info = copy.deepcopy(ped_locations)

    def cbOdom(self,msg):
        self.odom = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.psi = np.arctan2(2.0*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)) # bounded by [-pi, pi]
        print self.psi
        

    def cbControl(self,event):
        twist = self.action
        self.pub_twist.publish(twist)
        

    def cbComputeAction(self,event):
        current_state = [self.odom,self.ped_info]
        new_action = self.policy.predict(current_state)
        self.action = new_action

    def stopMoving(self):
        twist = Twist()
        self.pub_twist.publish(twist)
        
    def onShutdown(self):
        self.stopMoving()
        rospy.loginfo("Shutting down. Jackal has stopped moving.")
        


def main():

    rospy.init_node('huw_jackal_node', anonymous=False)
    print "Hello world from Jackal."
    jackal = Jackal()

    rospy.on_shutdown(jackal.onShutdown)

    rospy.spin()

if __name__ == '__main__':
    main()
