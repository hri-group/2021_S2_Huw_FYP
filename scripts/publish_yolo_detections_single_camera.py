#!/usr/bin/env python3

from pathlib import Path
import sys
try:
    sys.path.remove('/home/administrator/catkin_ws/devel/lib/python2.7/dist-packages')
    sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
except ValueError:
    pass
import cv2
import depthai as dai
import numpy as np
import time
import math

import argparse
import base64


'''
Spatial Tiny-yolo example
  Performs inference on RGB camera and retrieves spatial location coordinates: x,y,z relative to the center of depth map.
  Can be used for tiny-yolo-v3 or tiny-yolo-v4 networks
'''

# Tiny yolo v3/4 label texts
labelMap = [
    "person",         "bicycle",    "car",           "motorbike",     "aeroplane",   "bus",           "train",
    "truck",          "boat",       "traffic light", "fire hydrant",  "stop sign",   "parking meter", "bench",
    "bird",           "cat",        "dog",           "horse",         "sheep",       "cow",           "elephant",
    "bear",           "zebra",      "giraffe",       "backpack",      "umbrella",    "handbag",       "tie",
    "suitcase",       "frisbee",    "skis",          "snowboard",     "sports ball", "kite",          "baseball bat",
    "baseball glove", "skateboard", "surfboard",     "tennis racket", "bottle",      "wine glass",    "cup",
    "fork",           "knife",      "spoon",         "bowl",          "banana",      "apple",         "sandwich",
    "orange",         "broccoli",   "carrot",        "hot dog",       "pizza",       "donut",         "cake",
    "chair",          "sofa",       "pottedplant",   "bed",           "diningtable", "toilet",        "tvmonitor",
    "laptop",         "mouse",      "remote",        "keyboard",      "cell phone",  "microwave",     "oven",
    "toaster",        "sink",       "refrigerator",  "book",          "clock",       "vase",          "scissors",
    "teddy bear",     "hair drier", "toothbrush"
]

syncNN = True

# Get argument first
nnBlobPath = str((Path(__file__).parent / Path('models/tiny-yolo-v4_openvino_2021.2_6shave.blob')).resolve().absolute())
oak_camera_ids = ['14442C1071C48ED000','14442C10517660D700','14442C10D11C61D700']

parser = argparse.ArgumentParser(description='Inputs for OAK-D camera detection')
parser.add_argument('--nnBlobPath', type=str, help='Blob file for neural net detections', default=nnBlobPath)
parser.add_argument('--oak_camera_id',type=str,help='OAK-D MxID to run detections on')
parser.add_argument('--oak_camera_num',type=int,help='OAK-D camera index to run detections on', default=0)
parser.add_argument('--ros_tf_frame',type=str,help='tf frame that detections occur in', default='base_link')
parser.add_argument('--ros_output_topic',type=str,help='ros topic to output detections', default='/spencer/perception/detected_persons')
parser.add_argument('--visualize', type=bool, help='Blob file for neural net detections', default=True)
args = parser.parse_args()

if not Path(args.nnBlobPath).exists():
    raise FileNotFoundError(f'Required file/s not found, please run "{sys.executable} install_requirements.py"')

# Start defining a pipeline
pipeline = dai.Pipeline()

# Define a source - color camera
colorCam = pipeline.createColorCamera()
spatialDetectionNetwork = pipeline.createYoloSpatialDetectionNetwork()
monoLeft = pipeline.createMonoCamera()
monoRight = pipeline.createMonoCamera()
stereo = pipeline.createStereoDepth()

xoutRgb = pipeline.createXLinkOut()
xoutNN = pipeline.createXLinkOut()
xoutBoundingBoxDepthMapping = pipeline.createXLinkOut()
xoutDepth = pipeline.createXLinkOut()

xoutRgb.setStreamName("rgb")
xoutNN.setStreamName("detections")
xoutBoundingBoxDepthMapping.setStreamName("boundingBoxDepthMapping")
xoutDepth.setStreamName("depth")


colorCam.setPreviewSize(416, 416)
colorCam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
colorCam.setInterleaved(False)
colorCam.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoLeft.setBoardSocket(dai.CameraBoardSocket.LEFT)
monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoRight.setBoardSocket(dai.CameraBoardSocket.RIGHT)

# setting node configs
stereo.setConfidenceThreshold(255)

spatialDetectionNetwork.setBlobPath(nnBlobPath)
spatialDetectionNetwork.setConfidenceThreshold(0.5)
spatialDetectionNetwork.input.setBlocking(False)
spatialDetectionNetwork.setBoundingBoxScaleFactor(0.5)
spatialDetectionNetwork.setDepthLowerThreshold(100)
spatialDetectionNetwork.setDepthUpperThreshold(20000) # this is furthest away a detection will be allowed, otherwise outputs zero!
# Yolo specific parameters
spatialDetectionNetwork.setNumClasses(80)
spatialDetectionNetwork.setCoordinateSize(4)
spatialDetectionNetwork.setAnchors(np.array([10,14, 23,27, 37,58, 81,82, 135,169, 344,319]))
spatialDetectionNetwork.setAnchorMasks({ "side26": np.array([1,2,3]), "side13": np.array([3,4,5]) })
spatialDetectionNetwork.setIouThreshold(0.5)

# Create outputs

monoLeft.out.link(stereo.left)
monoRight.out.link(stereo.right)

colorCam.preview.link(spatialDetectionNetwork.input)
if syncNN:
    spatialDetectionNetwork.passthrough.link(xoutRgb.input)
else:
    colorCam.preview.link(xoutRgb.input)

spatialDetectionNetwork.out.link(xoutNN.input)
spatialDetectionNetwork.boundingBoxMapping.link(xoutBoundingBoxDepthMapping.input)

stereo.depth.link(spatialDetectionNetwork.inputDepth)
spatialDetectionNetwork.passthroughDepth.link(xoutDepth.input)

# start ROS node

import roslibpy

def convert_to_msg(x,y,z,confidence,detection_id):
    msg = {}
    # divide by 1000 as spencer expects m not mm. -1 * x as I think spatial data is given in a left handed coordinate system TODO: check this
    pose = {'position':{'x':-1*x/1000,'y':y/1000,'z':z/1000}, 'orientation':{'x':0,'y':0,'z':0,'w':0}} 
    msg['pose'] = {'pose':pose, 'covariance':[0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 999999999.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 999999.0, 999999.0, 999999.0, 0.0, 0.0, 0.0, 999999.0, 999999.0, 999999.0, 0.0, 0.0, 0.0, 999999.0, 999999.0, 999999.0]
}
    msg['modality'] = 'stereo'
    msg['confidence'] = confidence
    msg['detection_id'] = detection_id
    return msg


client = roslibpy.Ros(host='localhost', port=9090)
client.run()
talker_detections = roslibpy.Topic(client, args.ros_output_topic, 'spencer_tracking_msgs/DetectedPersons')
talker_image = roslibpy.Topic(client, '/oak' + str(args.oak_camera_num) + '/image/compressed', 'sensor_msgs/CompressedImage')

talker_detections.advertise()
talker_image.advertise()

if args.oak_camera_id is not None:
    device_found,device_info = dai.Device.getDeviceByMxId(args.oak_camera_id)
elif args.oak_camera_num is not None:
    device_found,device_info = dai.Device.getDeviceByMxId(oak_camera_ids[args.oak_camera_num])
else:
    device_found,device_info = dai.Device.getAnyAvailableDevice()
    

if not device_found:
    raise RuntimeError("Device not found!")

# Connect and start the pipeline
with dai.Device(pipeline,device_info) as device:

    # Output queues will be used to get the rgb frames and nn data from the outputs defined above
    previewQueue = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
    detectionNNQueue = device.getOutputQueue(name="detections", maxSize=4, blocking=False)
    xoutBoundingBoxDepthMapping = device.getOutputQueue(name="boundingBoxDepthMapping", maxSize=4, blocking=False)
    depthQueue = device.getOutputQueue(name="depth", maxSize=4, blocking=False)

    frame = None
    detections = []

    startTime = time.monotonic()
    counter = 0
    fps = 0
    color = (255, 255, 255)

    detection_count = 0
    img_count = 0

    while True:
        inPreview = previewQueue.get() #TODO get blocks until message is available, may be better using tryget
        inNN = detectionNNQueue.get()
        depth = depthQueue.get()     
        
        cur_time = time.time()
        secs = math.floor(cur_time)
        nsecs = (cur_time - secs)*10**9
        time_stamp = dict(secs=secs, nsecs=nsecs)

        if False:# args.oak_camera_num == 0:
            print(time_stamp)

        counter+=1
        img_count+=1

        current_time = time.monotonic()
        if (current_time - startTime) > 1 :
            fps = counter / (current_time - startTime)
            counter = 0
            startTime = current_time

        frame = inPreview.getCvFrame()
        depthFrame = depth.getFrame()

        depthFrameColor = cv2.normalize(depthFrame, None, 255, 0, cv2.NORM_INF, cv2.CV_8UC1)
        depthFrameColor = cv2.equalizeHist(depthFrameColor)
        depthFrameColor = cv2.applyColorMap(depthFrameColor, cv2.COLORMAP_HOT)
        detections = inNN.detections
        if len(detections) != 0:
            boundingBoxMapping = xoutBoundingBoxDepthMapping.get()
            roiDatas = boundingBoxMapping.getConfigData()

            for roiData in roiDatas:
                roi = roiData.roi
                roi = roi.denormalize(depthFrameColor.shape[1], depthFrameColor.shape[0])
                topLeft = roi.topLeft()
                bottomRight = roi.bottomRight()
                xmin = int(topLeft.x)
                ymin = int(topLeft.y)
                xmax = int(bottomRight.x)
                ymax = int(bottomRight.y)

                cv2.rectangle(depthFrameColor, (xmin, ymin), (xmax, ymax), color, cv2.FONT_HERSHEY_SCRIPT_SIMPLEX)


        # If the frame is available, draw bounding boxes on it and show the frame
        height = frame.shape[0]
        width  = frame.shape[1]

        person_msgs = []

        for detection in detections:
            # Denormalize bounding box
            x1 = int(detection.xmin * width)
            x2 = int(detection.xmax * width)
            y1 = int(detection.ymin * height)
            y2 = int(detection.ymax * height)
            try:
                label = labelMap[detection.label]
            except:
                label = detection.label
            if label == "person":
                if args.visualize:
                    cv2.putText(frame, str(label), (x1 + 10, y1 + 20), cv2.FONT_HERSHEY_TRIPLEX, 0.5, color)
                    cv2.putText(frame, "{:.2f}".format(detection.confidence*100), (x1 + 10, y1 + 35), cv2.FONT_HERSHEY_TRIPLEX, 0.5, color)
                    cv2.putText(frame, f"X: {int(detection.spatialCoordinates.x)} mm", (x1 + 10, y1 + 50), cv2.FONT_HERSHEY_TRIPLEX, 0.5, color)
                    cv2.putText(frame, f"Y: {int(detection.spatialCoordinates.y)} mm", (x1 + 10, y1 + 65), cv2.FONT_HERSHEY_TRIPLEX, 0.5, color)
                    cv2.putText(frame, f"Z: {int(detection.spatialCoordinates.z)} mm", (x1 + 10, y1 + 80), cv2.FONT_HERSHEY_TRIPLEX, 0.5, color)
                    cv2.rectangle(frame, (x1, y1), (x2, y2), color, cv2.FONT_HERSHEY_SIMPLEX)

                person_msgs.append(convert_to_msg(int(detection.spatialCoordinates.x),int(detection.spatialCoordinates.y),int(detection.spatialCoordinates.z),detection.confidence,detection_count))
                detection_count = detection_count + 1

        if args.visualize:
            cv2.putText(frame, "NN fps: {:.2f}".format(fps), (2, frame.shape[0] - 4), cv2.FONT_HERSHEY_TRIPLEX, 0.4, color)
            # cv2.imshow("depth " + str(device_info.getMxId()), depthFrameColor)
            # cv2.imshow("rgb " + str(device_info.getMxId()), frame)
            
        if client.is_connected and len(person_msgs)>0:
            talker_detections.publish(roslibpy.Message({'header': roslibpy.Header(seq=detection_count, frame_id=args.ros_tf_frame,stamp=time_stamp),'detections':person_msgs}))
        if client.is_connected and  args.visualize:
            frame_compressed = base64.b64encode(cv2.imencode('.jpg', frame)[1]).decode('ascii')
            talker_image.publish(dict(header=roslibpy.Header(seq=img_count, frame_id=args.ros_tf_frame,stamp=time_stamp),format='jpeg', data=frame_compressed))
            # print(person_msgs)

        time.sleep(0.1)

        if cv2.waitKey(1) == ord('q'):
            talker_detections.unadvertise()
            talker_image.unadvertise()
            client.terminate()
            break

