#!/bin/sh

echo "Make sure you have sourced your virtual environment running python3 with depthai installed"

echo "Enter oak camera id number"
read oak_id
echo Reading detections from oak $oak_id
echo Detections occur in oak_${oak_id}_frame 
echo Publishing detections to /spencer/perception_internal/detected_persons/oak_$oak_id

echo "Visualize images? (Y/n)"
read visualize

if [ "$visualize" = "Y" ]
then
python3 src/development/scripts/publish_yolo_detections_single_camera.py --oak_camera_num $oak_id --ros_tf_frame oak_${oak_id}_frame --ros_output_topic /spencer/perception_internal/detected_persons/oak_$oak_id --visualize True &
else
python3 src/development/scripts/publish_yolo_detections_single_camera.py --oak_camera_num $oak_id --ros_tf_frame oak_${oak_id}_frame --ros_output_topic /spencer/perception_internal/detected_persons/oak_$oak_id &
fi
