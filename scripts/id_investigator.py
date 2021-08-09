#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from spencer_tracking_msgs.msg import DetectedPersons
import numpy as np
import os



def callback(data):
    print(data.header.frame_id +'\n')
    for i in data.detections:
        print(str(i.detection_id) + ' ' + i.modality)
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/spencer/perception/detected_persons", DetectedPersons, callback)
    print 'node up'
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except:
        pass
