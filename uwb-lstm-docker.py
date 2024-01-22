#!/usr/bin/env python
# Common python libaries
import os 
import time
import math
import argparse
import numpy                    as np
import itertools
import copy

# ROS libaries
import rclpy
from rclpy.node                 import Node
from rclpy.qos                  import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg          import PoseStamped
from sensor_msgs.msg            import Range
from nav_msgs.msg               import Odometry

from tensorflow                 import keras



turtles         = ["4", "1", "2"  , "3", "5"]

uwbs            = ["4", "1", "2"  , "3", "5"]
uwb_pair        = [(4,1), (4,2), (4,3), (4,5), (1,2), (1,3), (1,5), (2,3), (2,5), (3,5)]

uwb_turtles     = [(0,1), (0,2), (0,3), (0,4), (1,2), (1,3), (1,4), (2,3), (2,4), (3,4)]
# uwb_odoms       = [(2,1), (3,1), (0,1), (2,3), (0,2), (0,3), (1,0), (2,0), (3,0), (1,0)]


#  get parameters from terminal
def parse_args():
    parser = argparse.ArgumentParser(description='Options to control relative localization with only UWB, assisit with Vision, and all if vision available')
    parser.add_argument('--with_model', type=utils.str2bool, default=False, help=' choose to model the uwb error or not')
    args = parser.parse_args()
    return args

args = parse_args()



class UWBLSTMRangeCorrection(Node):
    '''
        ROS Node that estimates relative position of two robots using odom and single uwb range.
    '''
    def __init__(self) :
        '''
            Define the parameters, publishers, and subscribers
        '''
        # Init node
        super().__init__('uwb-lstm-range-correction')
        # Define QoS profile for odom and UWB subscribers
        self.qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
   

        # all varibles 
        self.uwb_ranges             = [0.0 for _ in uwb_pair]
        self.turtles_mocaps         = [np.zeros(6) for _ in turtles]
  
        if args.with_model:
            self.models                 = [keras.models.load_model('/home/xianjia/Workspace/temp/lstm_ws/lstm_uwb_{}'.format(inx)) for inx in range(len(uwb_pair))]
            self.lstm_inputs            = [[] for _ in uwb_pair]
            self.n_steps                = 30
            self.uwb_lstm_ranges        = []
            self.uwb_real               = []
            self.uwb_inputs             = []
            time.sleep(1.0)

        self.get_logger().info("Subscribing to topics......")

        # subscribe to optitrack mocap poses
        self.mocap_subs = [
            self.create_subscription(PoseStamped, "/vrpn_client_node/tb0{}/pose".format(t), 
            self.create_mocap_pose_cb(i), qos_profile=self.qos) for i, t in enumerate(turtles)]
        self.get_logger().info("{} Mocaps poses subscribed!".format(len(self.turtles_mocaps)))

        # subscribe to uwb ranges 
        self.uwb_subs = [
            self.create_subscription(Range, "/uwb/tof/n_{}/n_{}/distance".format(p[0], p[1]), 
            self.create_uwb_ranges_cb(i),qos_profile=self.qos) for i, p in enumerate(uwb_pair)]
        self.get_logger().info("{} UWB ranges subscribed!".format(len(self.uwb_ranges)))

        self.uwb_publishers = [self.create_publisher(Range, "/corrected_uwb/tof/n_{}/n_{}/distance/replica".format(p[0], p[1]),  qos_profile=self.qos) for i, p in enumerate(uwb_pair)]

    def uwb_range_cb(self, i, range):
        self.uwb_ranges[i] = range.range
        # self.uwb_inputs = self.cal_lstm_input()
        if args.with_model:
            node1_mocap = self.turtles_mocaps[uwb_turtles[i][0]] 
            node2_mocap = self.turtles_mocaps[uwb_turtles[i][1]]
            node1_yaws = utils.euler_from_quaternion(np.array([node1_mocap[2], node1_mocap[3], node1_mocap[4],node1_mocap[5]]))
            node2_yaws = utils.euler_from_quaternion(np.array([node2_mocap[2], node2_mocap[3], node2_mocap[4],node2_mocap[5]]))
            self.lstm_inputs[i].append([self.uwb_ranges[i], node1_yaws, node2_yaws])
            if len(self.lstm_inputs[i]) > self.n_steps:
                lstm_input_arr = np.array(self.lstm_inputs[i][-self.n_steps:])
                bia = self.models[i].predict(np.reshape(lstm_input_arr,(1, self.n_steps, 3)), verbose = 0)
                self.uwb_ranges[i] = self.uwb_ranges[i] - bia[0]
            else:
                self.uwb_ranges[i] = self.uwb_ranges[i] 
        else:
            self.uwb_ranges[i] = self.uwb_ranges[i]
        # TODO: Add corrected range publisher here.
        

def main(args=None):
    rclpy.init(args=args)
    
    uwb_lstm_node = UWBLSTMRangeCorrection()

    time.sleep(1)
    # Start calculating relative positions
    
    uwb_lstm_node.get_logger().info("Starting LSTM Correction..")

    try:
        try:
            while rclpy.ok() :
                rclpy.spin(uwb_lstm_node)             
        except KeyboardInterrupt :
            uwb_lstm_node.get_logger().error('Keyboard Interrupt detected! Trying to stop filter node!')
    except Exception as e:
        uwb_lstm_node.destroy_node()
        uwb_lstm_node.get_logger().info("UWB LSTM Range Correction Failed %r."%(e,))
    finally:
        rclpy.shutdown()
        uwb_lstm_node.destroy_node()   

    

if __name__ == '__main__':
    main()          