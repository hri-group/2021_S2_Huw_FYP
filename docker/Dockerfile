# syntax=docker/dockerfile:1
FROM ros:noetic-robot

RUN apt-get update && apt-get install -y python3-pip git python3-catkin-tools && rm -rf /var/lib/apt/lists/*

RUN pip3 install Cython numpy

# UNCOMMENT THESE THREE LINESE
# RUN git clone https://github.com/sybrenstuvel/Python-RVO2 && cd Python-RVO2 && python3 setup.py build && python3 setup.py install
# RUN git clone https://github.com/ChanganVR/socialforce && cd socialforce && pip3 install -e '.[test,plot]'
# RUN git clone https://github.com/ChanganVR/RelationalGraphLearning && cd RelationalGraphLearning && pip3 install -e .

# clone ros package repo
ENV ROS_WS /opt/ros_ws
RUN mkdir -p $ROS_WS/src
WORKDIR $ROS_WS
# Clone package with the file we want
RUN git -C src clone -b $ROS_DISTRO-devel https://github.com/ros/ros_tutorials.git

# install ros package dependencies CHANGE
RUN apt-get update && rosdep update && rosdep install -y --from-paths src/ros_tutorials/roscpp_tutorials --ignore-src && rm -rf /var/lib/apt/lists/*

# build ros package source CHANGE
RUN catkin config --extend /opt/ros/$ROS_DISTRO && catkin build roscpp_tutorials

# source ros package from entrypoint
RUN sed --in-place --expression '$isource "$ROS_WS/devel/setup.bash"' /ros_entrypoint.sh

# run ros package launch file CHANGE
CMD ["roslaunch", "roscpp_tutorials", "talker_listener.launch"]