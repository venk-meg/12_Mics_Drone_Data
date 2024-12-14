#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float32MultiArray, Float32, String
from geometry_msgs.msg import PoseStamped
import os
import csv
import threading
import datetime

#store db, distance data in csv file.
class SoundDistanceSubscriber(Node):
    def __init__(self):
        super().__init__('sound_distance_subscriber')

        #Get launch parameters
        self.declare_parameter('numberofmics')
        self.numberofmics = self.get_parameter('numberofmics').value

        #tello pose subscription
        self.tello_pose_sub = self.create_subscription(PoseStamped, '/vrpn_client_node/Tello/pose', self.tello_pose_callback, 10)

        #mics pose subscription
        #audio dblevel subscription
        self.mic_subs = []
        self.db_subs = []
        self.timestamp_subs = []

        self.reference_amplitude = 1

        #create subscribers
        for i in range(self.numberofmics):
            mictopicname = f'/vrpn_client_node/Microphone{i+1}/pose'
            dbtopicname = f'db_data{i}'
            timestamp_topicname = f'timestamp{i}'
            self.mic_subs.append(self.create_subscription(PoseStamped, mictopicname, lambda msg, mic_index=i: self.mic_pose_callback(msg, mic_index), 10))
            self.db_subs.append(self.create_subscription(Float32MultiArray, dbtopicname, lambda msg, mic_index=i: self.db_callback(msg, mic_index), 10))
            self.timestamp_subs.append(self.create_subscription(String, timestamp_topicname, lambda msg, mic_index=i: self.timestamp_callback(msg, mic_index), 10))


        #create csv logger
        self.timer = self.create_timer(0.1, self.timer_callback)

        #initialise variables
        self.tello_position = None
        self.mic_position_logger = [None]*self.numberofmics
        self.mic_position_frozen = [None]*self.numberofmics
        self.mic_distance = [None]*self.numberofmics
        self.db_levels = [0.0]*self.numberofmics
        self.timestamps = [None]*self.numberofmics

        #for csv writing
        data_folder = '/home/idris/ros2_ws/drone_data/drone_data_12' #directory where files are being saved
        entries = os.listdir(data_folder)
        csv_count = len([entry for entry in entries if os.path.isfile(os.path.join(data_folder, entry))])
        currentdateandtime = datetime.datetime.now()
        self.csv_file = open(os.path.join(data_folder, f'12mics_experiment_normalnocaps50_{csv_count}_{currentdateandtime}.csv'), 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        #write header row
        self.csv_writer.writerow(['Timestamp', 'tello_pos',
                                  
                                  'mic1_pos', 'mic2_pos', 'mic3_pos', 'mic4_pos', 
                                  'mic5_pos', 'mic6_pos', 'mic7_pos', 'mic8_pos', 
                                  'mic9_pos', 'mic10_pos', 'mic11_pos', 'mic12_pos', 

                                  'mic1_dist', 'mic2_dist', 'mic3_dist', 'mic4_dist', 
                                  'mic5_dist', 'mic6_dist', 'mic7_dist', 'mic8_dist', 
                                  'mic9_dist', 'mic10_dist', 'mic11_dist', 'mic12_dist', 
                               
                                  'mic1_db', 'mic2_db', 'mic3_db', 'mic4_db', 
                                  'mic5_db', 'mic6_db', 'mic7_db', 'mic8_db', 
                                  'mic9_db', 'mic10_db', 'mic11_db', 'mic12_db',
                                  
                                  'mic1_timestamp', 'mic2_timestamp', 'mic3_timestamp', 'mic4_timestamp', 
                                  'mic5_timestamp', 'mic6_timestamp', 'mic7_timestamp', 'mic8_timestamp', 
                                  'mic9_timestamp', 'mic10_timestamp', 'mic11_timestamp', 'mic12_timestamp'])
        
        #threading
        self.lock = threading.Lock()
        

    def timer_callback(self):
        if self.tello_position is not None and all(position is not None for position in self.mic_position_frozen):
            self.calculate_distance() #calculates all disances and updates self.mic_distance
            
            with self.lock:
                row = [self.get_clock().now().to_msg(), self.tello_position]
                row.extend(self.mic_position_frozen)
                row.extend(self.mic_distance)
                row.extend(self.db_levels)
                row.extend(self.timestamps)
                self.csv_writer.writerow(row)
            
            # self.get_logger().info(f'logged row {row}')

    #Get Quadrotor position from vrpn server
    def tello_pose_callback(self,msg):
        self.tello_position = np.round(np.array([msg.pose.position.x,msg.pose.position.y,msg.pose.position.z]), decimals = 2)
        self.get_logger().info(f"telloposition = {self.tello_position}")

    #Get Microphone position from vrpn server
    def mic_pose_callback(self, msg, mic_index):
        message = np.round(np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]), decimals = 2)
        self.mic_position_logger[mic_index] = message
        if all(position is not None for position in self.mic_position_logger):  # Check if all positions are received
            self.get_logger().info("subscribed to all 12 mics")
            self.mic_position_frozen = self.mic_position_logger
            self.mic_position_logger = [None] * self.numberofmics

    #Get audio data from buffer subscriber
    def db_callback(self, msg, mic_index):
        data_array = np.array(msg.data)
        if len(data_array) > 0:
            rms_amplitude = np.sqrt(np.mean(np.square(data_array)))
            db = -np.inf if rms_amplitude == 0 else 20 * np.log10(rms_amplitude / self.reference_amplitude)
            self.db_levels[mic_index] = float(db)
        else:
            self.db_levels[mic_index] = np.nan #placeholder for invalid data
            self.get_logger().info(f"db data for mic{mic_index+1} not obtained") 

    def timestamp_callback(self, msg, mic_index):
        self.timestamps[mic_index] = msg.data
        # self.get_logger().info(f"timestamp data for mic{mic_index+1} not obtained") 
    
    def calculate_distance(self):
        for ind, mic_position in enumerate(self.mic_position_frozen):
            self.mic_distance[ind] = np.round(np.linalg.norm(self.tello_position - mic_position), decimals = 2)

    def on_shutdown(self):
        self.csv_file.close()

def main(args=None):
    rclpy.init(args=args)
    sound_dist_sub = SoundDistanceSubscriber()
    rclpy.spin(sound_dist_sub)
    sound_dist_sub.on_shutdown()
    sound_dist_sub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()