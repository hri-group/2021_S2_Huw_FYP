import time

import roslibpy
from crowd_sim.envs.utils.state import JointState, FullState, ObservableState
import rospy
from transformations import quaternion_from_euler,euler_from_quaternion
import numpy as np
import copy
#--------------------------------------------------------------------
# from test
import logging
import argparse
import importlib.util
import os
import torch
import numpy as np
import matplotlib.pyplot as plt
import gym
from crowd_nav.utils.explorer import Explorer
from crowd_nav.policy.policy_factory import policy_factory
from crowd_sim.envs.utils.robot import Robot
from crowd_sim.envs.policy.orca import ORCA

class Operate:
    def __init__(self):
        
        self.robot_radius = 1.0
        self.ped_radius = 1.0
        self.robot_v_pref = 1.2

        # roslibpy
        # this assumes that the all the messages are published in the same frame, ok if odom is fixed/map frame but what if not?
        self.client = roslibpy.Ros(host='localhost', port=9090)
        self.client.run()
        
        self.robot_listener = roslibpy.Topic(self.client, '/odometry/filtered', 'nav_msgs/Odometry')
        self.robot_listener.subscribe(self.cbRobot)

        self.person_listener = roslibpy.Topic(self.client, '/pedsim_visualizer/tracked_persons', 'pedsim_msgs/TrackedPersons')
        self.person_listener.subscribe(self.cbPersons)

        self.goal_listener = roslibpy.Topic(self.client, '/move_base_simple/goal', 'geometry_msgs/PoseStamped')
        self.goal_listener.subscribe(self.cbGoal)

        self.vel_publisher = roslibpy.Topic(self.client, '/cmd_vel', 'geometry_msgs/Twist')


        # stored robot and person poses
        self.robot_pose = FullState(0,0,0,0,self.robot_radius,0,0,self.robot_v_pref,0)
        self.person_poses = []



    def cbRobot(self,msg):
        self.robot_pose.px = float(msg['pose']['pose']['position']['x'])
        self.robot_pose.py = float(msg['pose']['pose']['position']['y'])
        euler_angles = euler_from_quaternion([float(msg['pose']['pose']['orientation']['x']),float(msg['pose']['pose']['orientation']['y']),float(msg['pose']['pose']['orientation']['z']),float(msg['pose']['pose']['orientation']['w'])])
        self.robot_pose.theta = euler_angles[2]
        
        linear_velocity = float(msg['twist']['twist']['linear']['x'])
        self.robot_pose.vx =  linear_velocity * np.cos(self.robot_pose.theta)
        self.robot_pose.vy = linear_velocity * np.sin(self.robot_pose.theta)
        # print([self.robot_pose.vx,self.robot_pose.vy])

    def cbPersons(self,msg):
        
        persons_poses = []
        for track in msg['tracks']:
            px = float(track['pose']['pose']['position']['x'])
            py = float(track['pose']['pose']['position']['y'])
            vx = float(track['twist']['twist']['linear']['x'])
            vy = float(track['twist']['twist']['linear']['y'])
            person = ObservableState(px,py,vx,vy,self.ped_radius)
            persons_poses.append(person)
        self.person_poses = copy.deepcopy(persons_poses)
        

    def cbGoal(self,msg):
        self.robot_pose.gx = float(msg['pose']['position']['x'])
        self.robot_pose.gy = float(msg['pose']['position']['y'])
        print([self.robot_pose.gx,self.robot_pose.gy])

    def publish_action(self,action):
        msg = {
                "linear": {"x": action.v,"y": 0.0,"z": 0.0},
                "angular": {"x": 0.0,"y": 0.0,"z": action.r},
                }
        self.vel_publisher.publish(msg)




def main(args):

    # configure logging and device
    level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(level=level, format='%(asctime)s, %(levelname)s: %(message)s',
                        datefmt="%Y-%m-%d %H:%M:%S")
    device = torch.device("cuda:0" if torch.cuda.is_available() and args.gpu else "cpu")
    logging.info('Using device: %s', device)

    if args.model_dir is not None:
        if args.config is not None:
            config_file = args.config
        elif args.model_folder_config is not None:
            config_file = os.path.join(args.model_dir, args.model_folder_config)
        else:
            config_file = os.path.join(args.model_dir, 'config.py')
        if args.il:
            model_weights = os.path.join(args.model_dir, 'il_model.pth')
            logging.info('Loaded IL weights')
        elif args.rl:
            if os.path.exists(os.path.join(args.model_dir, 'resumed_rl_model.pth')):
                model_weights = os.path.join(args.model_dir, 'resumed_rl_model.pth')
            else:
                print(os.listdir(args.model_dir))
                model_weights = os.path.join(args.model_dir, sorted(os.listdir(args.model_dir))[-1])
            logging.info('Loaded RL weights')
        else:
            model_weights = os.path.join(args.model_dir, 'best_val.pth')
            logging.info('Loaded RL weights with best VAL')

    else:
        config_file = args.config

    spec = importlib.util.spec_from_file_location('config', config_file)
    if spec is None:
        parser.error('Config file not found.')
    config = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(config)

    # configure policy
    policy_config = config.PolicyConfig(args.debug)
    policy = policy_factory[policy_config.name]()
    if args.planning_depth is not None:
        policy_config.model_predictive_rl.do_action_clip = True
        policy_config.model_predictive_rl.planning_depth = args.planning_depth
    if args.planning_width is not None:
        policy_config.model_predictive_rl.do_action_clip = True
        policy_config.model_predictive_rl.planning_width = args.planning_width
    if args.sparse_search:
        policy_config.model_predictive_rl.sparse_search = True

    policy.configure(policy_config)
    policy.set_device(device)
    if policy.trainable:
        if args.model_dir is None:
            parser.error('Trainable policy must be specified with a model weights directory')
        policy.load_model(model_weights)

    # configure environment
    env_config = config.EnvConfig(args.debug)

    if args.human_num is not None:
        env_config.sim.human_num = args.human_num
    env = gym.make('CrowdSim-v0')
    env.configure(env_config)
    env.set_human_safety_space(getattr(env_config, 'test').human_safety_space) # Override training environemnt human safety space setting

    if args.square:
        env.test_scenario = 'square_crossing'
    if args.circle:
        env.test_scenario = 'circle_crossing'
    if args.test_scenario is not None:
        env.test_scenario = args.test_scenario

    robot = Robot(env_config, 'robot')
    robot.visible = getattr(env_config, 'test').robot_visible # Override training environemnt robot visible setting
    env.set_robot(robot)
    robot.time_step = env.time_step
    robot.set_policy(policy)
    explorer = Explorer(env, robot, device, None, gamma=0.9)

    train_config = config.TrainConfig(args.debug)
    epsilon_end = train_config.train.epsilon_end
    if not isinstance(robot.policy, ORCA):
        robot.policy.set_epsilon(epsilon_end)

    policy.set_phase(args.phase)
    # set safety space for ORCA in non-cooperative simulation
    if isinstance(robot.policy, ORCA):
        if robot.visible:
            robot.policy.safety_space = args.safety_space
        else:
            robot.policy.safety_space = args.safety_space
        logging.info('ORCA agent buffer: %f', robot.policy.safety_space)

    policy.set_env(env)
    robot.print_info()

    # -------------------------------------------------------------------------------------------------------------
    # everything above here is copied from the test.py file, used to make sure the policy and environment is configured correctly. TODO: tidy up and only import policy. 
    # where the magic happens
    policy.query_env = False
    policy.kinematics = 'unicycle'
    policy.set_time_step(0.01)
    operate = Operate()

    
    try:
        while True:
            state = JointState(operate.robot_pose, operate.person_poses)
            # print([operate.robot_pose.px,operate.robot_pose.py,operate.robot_pose.vx,operate.robot_pose.vy,operate.robot_pose.theta])
            action = robot.policy.predict(state) # the only thing we actually need from RGL, TODO: isolate this rather than using whole test.py script. 
            operate.publish_action(action)
            time.sleep(0.01)
    except KeyboardInterrupt:
        operate.client.terminate()


        

 


if __name__ == '__main__':
    parser = argparse.ArgumentParser('Parse configuration file')
    parser.add_argument('--config', type=str, default=None)
    parser.add_argument('--policy', type=str, default='model_predictive_rl')
    parser.add_argument('-m', '--model_dir', type=str, default=None)
    parser.add_argument('--il', default=False, action='store_true')
    parser.add_argument('--rl', default=False, action='store_true')
    parser.add_argument('--gpu', default=False, action='store_true')
    parser.add_argument('-v', '--visualize', default=False, action='store_true')
    parser.add_argument('--phase', type=str, default='test')
    parser.add_argument('-c', '--test_case', type=int, default=None)
    parser.add_argument('--square', default=False, action='store_true')
    parser.add_argument('--circle', default=False, action='store_true')
    parser.add_argument('--video_file', type=str, default=None)
    parser.add_argument('--video_dir', type=str, default=None)
    parser.add_argument('--traj', default=False, action='store_true')
    parser.add_argument('--debug', default=False, action='store_true')
    parser.add_argument('--human_num', type=int, default=None)
    parser.add_argument('--safety_space', type=float, default=0.2)
    parser.add_argument('--test_scenario', type=str, default=None)
    parser.add_argument('-d', '--planning_depth', type=int, default=None)
    parser.add_argument('-w', '--planning_width', type=int, default=None)
    parser.add_argument('--sparse_search', default=False, action='store_true')
    
    parser.add_argument('-n', '--case_count', type=int, default=None)
    parser.add_argument('--plot_test_scenarios_hist', default=False, action='store_true')
    parser.add_argument('--output_data_file_name', default=None, type=str)
    parser.add_argument('--model_folder_config', default=None, type=str)

    sys_args = parser.parse_args()

    main(sys_args)