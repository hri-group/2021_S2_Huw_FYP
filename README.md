# 2021_S2_Huw_FYP

This project focussed on developing a detection and tracking pipeline to detect pedestrians near the Jackal robot. The CADRL navigation algorithm developed by MIT was tested in simulation and onboard the robot. 

## Test algorithm in Gazebo simulator example

1. Source workspace e.g. `source devel/setup.bash`
2. `roslaunch development test_mit.launch`. This launch file will start up the jackal gazebo simulator, launch pedsim to simulate pedestrians, and start the mit_cadrl node for navigation. To test with your own navigation node set the cadrl_node argument to false. 

## Test algorithm on Jackal robot

1. Launch the lidar person detector that runs in a Docker container. The docker file can be found in the docker_dr_spaam folder. To build navigate to the folder and run `docker build --tag dr_spaam .` It is already built on Jackal 1. To execute run `docker run -t --rm --net=host dr_spaam`. This will start the person detector using scans from the lidar detector. The detector's input and output topics are set in [this](docker_dr_spaam/config/topics.yaml) config file.
  
2. `roslaunch development test_mit_on_jackal.launch` This will launch the spencer tracking pipeline, which aggregates and fuses detections from the enabled lidar and camera detectors, then passes the detections to a nearest neighbour tracker. The tracker outputs trackedPerson information on the /spencer/perception/tracked_persons topic. The launch also starts the mit_cadrl navigation node. To test with your own navigation algorithm set the cadrl_node argument to false. 

## Dependencies

### Simulation

1. [Pedsim](https://github.com/srl-freiburg/pedsim_ros)
2. [Jackal Simulator](https://github.com/jackal/jackal_simulator). Overview and Installation instructions available [here](https://www.clearpathrobotics.com/assets/guides/kinetic/jackal/simulation.html)
3. If using the mit cadrl navigation node install from [here](https://github.com/mit-acl/cadrl_ros).

### On the robot
1. [Spencer](https://github.com/hri-group/spencer_people_tracking)
2. [DepthAI](https://github.com/luxonis/depthai-ros)
3. [DepthAI examples](https://github.com/hri-group/depthai-ros-examples)

DepthAI is the interface used for communicating with the OAK-D cameras. 
