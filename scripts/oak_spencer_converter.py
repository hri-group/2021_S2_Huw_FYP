#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose, PoseStamped, Twist, Vector3,Vector3Stamped,PoseWithCovariance,TwistWithCovariance,Point
import tf
from spencer_tracking_msgs.msg import DetectedPerson, DetectedPersons
from depthai_ros_msgs.msg import SpatialDetectionArray

class oak_spencer_helper():
    def __init__(self):

        # tf listener
        self.detection_id = 0

        self.listener = tf.TransformListener()

        # converting oak-d detections to spencer detected people 
        self.sub_oak_detections = rospy.Subscriber('~oak_detections', SpatialDetectionArray, self.cbOak)
        self.pub_spencer_detections = rospy.Publisher('~spencer_detections',DetectedPersons,queue_size=1)

    def cbOak(self,msg):
        ped_locations = DetectedPersons()
        ped_locations.header = msg.header
        for i in xrange(len(msg.detections)):
            if msg.detections[i].results[0].id==0: # if id = 0 detection is a person
                ped_location = DetectedPerson()
                self.detection_id+=1
                ped_location.detection_id = self.detection_id
                ped_location.confidence = msg.detections[i].results[0].score
                ped_location.pose.pose.position = msg.detections[i].position 
                ped_location.pose.covariance = [0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 999999999.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 999999.0, 999999.0, 999999.0, 0.0, 0.0, 0.0, 999999.0, 999999.0, 999999.0, 0.0, 0.0, 0.0, 999999.0, 999999.0, 999999.0]
                ped_location.modality = 'stereo'
                ped_locations.detections.append(ped_location)


        self.pub_spencer_detections.publish(ped_locations)            

if __name__ == '__main__':
    rospy.init_node('oak_spencer_helper', anonymous=True)
    print "Topic converter online."
    helper = oak_spencer_helper()
    rospy.spin()
