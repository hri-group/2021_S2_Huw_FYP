# syntax=docker/dockerfile:1
FROM ros:noetic-robot

RUN apt-get update && apt-get install -y python3-pip git python3-catkin-tools libgl1
RUN pip3 install torch==1.9.1+cpu torchvision==0.10.1+cpu torchaudio==0.9.1 -f https://download.pytorch.org/whl/torch_stable.html && pip3 install scipy opencv-python
RUN rm -rf /var/lib/apt/lists/*

# clone ros package repo
ENV ROS_WS /opt/ros_ws
RUN mkdir -p $ROS_WS/src
WORKDIR $ROS_WS
# Clone package with the file we want
RUN git -C src clone https://github.com/hri-group/2D_lidar_person_detection.git && cd src/2D_lidar_person_detection/dr_spaam && python3 setup.py install

# Install spencer messages for output pedestrian message format
RUN git -C src clone https://github.com/spencer-project/spencer_messages.git

# Replace default config files with custom files
RUN cd src/2D_lidar_person_detection/dr_spaam_ros/config && rm -r * 

COPY config src/2D_lidar_person_detection/dr_spaam_ros/config

COPY ckpt_jrdb_ann_ft_drow3_e40.pth .


# install ros package dependencies CHANGE
RUN apt-get update && rosdep update && rosdep install -y --from-paths src/2D_lidar_person_detection/dr_spaam_ros --ignore-src && rosdep install -y --from-paths src/spencer_messages --ignore-src && rm -rf /var/lib/apt/lists/*

# build ros package source CHANGE
RUN catkin config --extend /opt/ros/$ROS_DISTRO && catkin build dr_spaam_ros spencer_tracking_msgs

# source ros package from entrypoint
RUN sed --in-place --expression '$isource "$ROS_WS/devel/setup.bash"' /ros_entrypoint.sh
RUN ln -s /usr/bin/python3 /usr/bin/python

# run ros package launch file CHANGE
CMD ["roslaunch", "dr_spaam_ros", "dr_spaam_ros.launch"]
