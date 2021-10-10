#!/usr/bin/env python3

import base64
import logging
import time

import cv2
import numpy as np
import roslibpy

# Configure logging
fmt = '%(asctime)s %(levelname)8s: %(message)s'
logging.basicConfig(format=fmt, level=logging.INFO)
log = logging.getLogger(__name__)

client = roslibpy.Ros(host='localhost', port=9090)
client.run()

publisher = roslibpy.Topic(client, '/oak0/camera_info', 'sensor_msgs/CameraInfo')
publisher.advertise()

i = 0

vid = cv2.VideoCapture(0)

while(True):

      
    # Capture the video frame
    # by frame
    i+=1
    ret, frame = vid.read()

    jpg_frame = base64.b64encode(cv2.imencode('.jpg', frame)[1]).decode('ascii')
    talker_detections.publish(roslibpy.Message({'header': roslibpy.Header(seq=i, stamp=None, frame_id='base_link'),'distortion_model':'plumb_bob',}))
    
    publisher.publish(dict(format='jpeg', data=jpg_frame))
    
      
    # the 'q' button is set as the
    # quitting button you may use any
    # desired button of your choice
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
  
# After the loop release the cap object
vid.release()
# Destroy all the windows
cv2.destroyAllWindows()


                
                
                


