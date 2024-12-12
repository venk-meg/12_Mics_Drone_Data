#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray
from geometry_msgs.msg import PoseStamped
import numpy as np
import sounddevice as sd
import csv
import threading
import os

class SoundAndDistancePublisher(Node):
    def __init__(self,filename='experiment'):
        super().__init__('sound_distance12_publisher')

        self.publisher_ = self.create_publisher(Float32MultiArray, 'audio_distance_data12', 10)
        self.tello_pose_sub = self.create_subscription(PoseStamped, '/vrpn_client_node/Tello/pose', self.tello_pose_callback, 10)
        
        self.mic_pose_sub = []
        for i in range(12):
            self.mic_pose_sub.append(self.create_subscription(PoseStamped, f'/vrpn_client_node/Microphone{i+1}/pose', self.mic_pose_callback, 10))
        # self.mic_pose_sub = self.create_subscription(PoseStamped, '/vrpn_client_node/Microphone1/pose', self.mic_pose_callback, 10)
        # self.mic_pose_sub2 = self.create_subscription(PoseStamped, '/vrpn_client_node/Microphone2/pose', self.mic_pose_callback, 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.tello_position = None
        self.mic_position = []
        # self.mic_position = None
        # self.mic_position2 = None

        self.duration = 0.1  # seconds
        self.sample_rate = 44100  # Appropriate sample rate for your mic
        self.reference_amplitude = 1.0  # Adjust based on needs

        self.mic_device_indices = [2,10,11,12,13,14,15,16,17,18,19,20] #replace with correct index of mics
        # self.device_index_mic1 = 2  # Replace with the correct index for mic 1
        # self.device_index_mic2 = 10  # Replace with the correct index for mic 2

        data_folder = '/home/idris/ros2_ws/drone_data/' #directory where csv files are being saved
        entries = os.listdir(data_folder)
        csv_count = len([entry for entry in entries if os.path.isfile(os.path.join(data_folder, entry))]) #give each csv a unique number
        #self.csv_file = open(data_folder+f'/audio_distance_log_combine_{filename}_{csv_count}.csv', 'w', newline='')
        self.csv_file = open(data_folder+filename+f'_{csv_count}.csv', 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)


        self.csv_writer.writerow(['Timestamp', 'mic1_pos', 'mic2_pos', 'mic3_pos', 'mic4_pos', 'mic5_pos', 'mic6_pos', 'mic7_pos', 'mic8_pos', 'mic9_pos', 'mic10_pos', 'mic11_pos', 'mic12_pos', 'tello_pos', 'Distance1', 'Distance2', 'Distance3', 'Distance4', 'Distance5', 'Distance6', 'Distance7', 'Distance8', 'Distance9', 'Distance10', 'Distance11', 'Distance12', '1 Sound Level (dB)', '2 Sound Level (dB)', '3 Sound Level (dB)', '4 Sound Level (dB)', '5 Sound Level (dB)', '6 Sound Level (dB)', '7 Sound Level (dB)', '8 Sound Level (dB)', '9 Sound Level (dB)', '10 Sound Level (dB)', '11 Sound Level (dB)', '12 Sound Level (dB)'])


        self.lock = threading.Lock()  # Lock to manage shared resources

    def timer_callback(self):
        #print(self.mic_position2)
        # if self.tello_position is not None and self.mic_position is not None and self.mic_position2 is not None:
        distances = []
        mic_threads = []
        if self.tello_position is not None and len(self.mic_position) > 0:
            for i in range(len(self.mic_position)):
                distances.append(np.linalg.norm(self.tello_position - self.mic_position[i]))
                # distance1 = np.linalg.norm(self.tello_position - self.mic_position)
                # distance2 = np.linalg.norm(self.tello_position - self.mic_position2)

                # Create and start threads for recording from each microphone
                mic_threads.append(threading.Thread(target=self.record_mic, args=(self.mic_device_indices[i], f'mic{i+1}')))
                # mic1_thread = threading.Thread(target=self.record_mic, args=(self.device_index_mic1, 'mic1'))
                # mic2_thread = threading.Thread(target=self.record_mic, args=(self.device_index_mic2, 'mic2'))

                # mic1_thread.start()
                # mic2_thread.start()
                mic_threads[i].start()

                # mic1_thread.join()
                # mic2_thread.join()
                mic_threads[i].join()

                #print("HELLO")

            msg = Float32MultiArray()
            msg.data = [float(self.db_mic1), float(self.db_mic2), float(self.db_mic3), float(self.db_mic4), float(self.db_mic5), float(self.db_mic6), float(self.db_mic7), float(self.db_mic8), float(self.db_mic9), float(self.db_mic10),float(self.db_mic11), float(self.db_mic12)]
            self.publisher_.publish(msg)

            #print("HELLO")

            # Log to CSV
            with self.lock:  # Ensure thread-safe writing to the file
                self.csv_writer.writerow([self.get_clock().now().to_msg(), 
                                          np.round(self.mic_position[0], decimals = 2), np.round(self.mic_position[1], decimals = 2), np.round(self.mic_position[2], decimals = 2), np.round(self.mic_position[3], decimals = 2), np.round(self.mic_position[4], decimals = 2), np.round(self.mic_position[5], decimals = 2), 
                                          np.round(self.mic_position[6], decimals = 2), np.round(self.mic_position[7], decimals = 2), np.round(self.mic_position[8], decimals = 2), np.round(self.mic_position[9], decimals = 2), np.round(self.mic_position[10], decimals = 2), np.round(self.mic_position[11], decimals = 2),
                                          np.round(self.tello_position, decimals = 2), 
                                          f"{distances[0]:.2f}", f"{distances[1]:.2f}", f"{distances[2]:.2f}", f"{distances[3]:.2f}", f"{distances[4]:.2f}", f"{distances[5]:.2f}", 
                                          f"{distances[6]:.2f}", f"{distances[7]:.2f}", f"{distances[8]:.2f}", f"{distances[9]:.2f}", f"{distances[10]:.2f}", f"{distances[11]:.2f}", 
                                          f"{self.db_mic1:.2f}", f"{self.db_mic2:.2f}", f"{self.db_mic3:.2f}", f"{self.db_mic4:.2f}", f"{self.db_mic5:.2f}", f"{self.db_mic6:.2f}", 
                                          f"{self.db_mic7:.2f}", f"{self.db_mic8:.2f}", f"{self.db_mic9:.2f}", f"{self.db_mic10:.2f}", f"{self.db_mic11:.2f}", f"{self.db_mic12:.2f}"])
            
            self.get_logger().info(f'Published Distance 1: {distances[0]:.2f} meters, Published Distance 2: {distances[1]:.2f} meters, Published Distance 3: {distances[2]:.2f} meters, Published Distance 4: {distances[3]:.2f} meters, Published Distance 5: {distances[4]:.2f} meters, Published Distance 6: {distances[5]:.2f} meters, Published Distance 7: {distances[6]:.2f} meters, Published Distance 8: {distances[7]:.2f} meters, Published Distance 9: {distances[8]:.2f} meters, Published Distance 10: {distances[9]:.2f} meters, Published Distance 11: {distances[10]:.2f} meters, Published Distance 12: {distances[11]:.2f} meters, Sound Level 1: {self.db_mic1:.4f} dB, Sound Level 2: {self.db_mic2:.4f} dB, Sound Level 3: {self.db_mic3:.4f} dB, Sound Level 4: {self.db_mic4:.4f} dB, Sound Level 5: {self.db_mic5:.4f} dB, Sound Level 6: {self.db_mic6:.4f} dB, Sound Level 7: {self.db_mic7:.4f} dB, Sound Level 8: {self.db_mic8:.4f} dB, Sound Level 9: {self.db_mic9:.4f} dB, Sound Level 10: {self.db_mic10:.4f} dB, Sound Level 11: {self.db_mic11:.4f} dB, Sound Level 12: {self.db_mic12:.4f} dB')

    def record_mic(self, device_index, mic_label):
        try:
            # Record from the specified microphone
            audio_data = sd.rec(int(self.duration * self.sample_rate), samplerate=self.sample_rate, channels=1, dtype='float32', device=device_index)
            sd.wait()

            # Calculate RMS and dB level
            rms_amplitude = np.sqrt(np.mean(np.square(audio_data)))
            db = 20 * np.log10(rms_amplitude / self.reference_amplitude)

            # Assign the dB value to the appropriate attribute
            with self.lock:
                if mic_label == 'mic_1':
                    self.db_mic1 = db
                elif mic_label == 'mic_2':
                    self.db_mic2 = db
                elif mic_label == 'mic_3':
                    self.db_mic3 = db
                elif mic_label == 'mic_4':
                    self.db_mic4 = db
                elif mic_label == 'mic_5':
                    self.db_mic5 = db
                elif mic_label == 'mic_6':
                    self.db_mic6 = db
                elif mic_label == 'mic_7':
                    self.db_mic7 = db
                elif mic_label == 'mic_8':
                    self.db_mic8 = db
                elif mic_label == 'mic_9':
                    self.db_mic9 = db
                elif mic_label == 'mic_10':
                    self.db_mic10 = db
                elif mic_label == 'mic_11':
                    self.db_mic11 = db
                elif mic_label == 'mic_12':
                    self.db_mic12 = db

                # if mic_label == 'mic1':
                #     self.db_mic1 = db
                # elif mic_label == 'mic2':
                #     self.db_mic2 = db

        except Exception as e:
            self.get_logger().error(f"Failed to capture audio on {mic_label}: {str(e)}")

    def tello_pose_callback(self, msg):
        self.tello_position = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])

    def mic_pose_callback(self, msg):
        self.mic_position.append(np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]))

    # def mic_pose_callback2(self, msg):
    #     self.mic_position2 = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])

    def on_shutdown(self):
        self.csv_file.close()

def main(args=None):
    rclpy.init(args=args)
    node = SoundAndDistancePublisher()
    rclpy.spin(node)
    node.on_shutdown()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

#!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Float32, Float32MultiArray
# from geometry_msgs.msg import PoseStamped
# import numpy as np
# import sounddevice as sd
# import argparse
# import csv
# import os

# device1 = 2
# device2 = 10

# class SoundAndDistancePublisher(Node):
#     def __init__(self, filename='experiment'):
#         super().__init__('sound_distance_publisher')
#         self.publisher_ = self.create_publisher(Float32MultiArray, 'audio_distance_data', 10)
#         self.tello_pose_sub = self.create_subscription(PoseStamped, '/vrpn_client_node/Tello/pose', self.tello_pose_callback, 10)
#         self.mic_pose_sub = self.create_subscription(PoseStamped, '/vrpn_client_node/Microphone1/pose', self.mic_pose_callback, 10)
#         self.mic_pose_sub2 = self.create_subscription(PoseStamped, '/vrpn_client_node/Microphone2/pose', self.mic_pose_callback2, 10)
#         self.timer = self.create_timer(0.1, self.timer_callback)
        
#         self.tello_position = None
#         self.mic_position = None
#         self.mic_position2 = None
#         self.duration = 0.1  # seconds
#         self.reference_amplitude = 1.0  # Adjust based on needs
#         data_folder = '/home/idris/ros2_ws/drone_data/' #directory where csv files are being saved
#         entries = os.listdir(data_folder)
#         csv_count = len([entry for entry in entries if os.path.isfile(os.path.join(data_folder, entry))]) #give each csv a unique number
#         #self.csv_file = open(data_folder+f'/audio_distance_log_combine_{filename}_{csv_count}.csv', 'w', newline='')
#         self.csv_file = open(data_folder+filename+f'_{csv_count}.csv', 'w', newline='')
#         self.csv_writer = csv.writer(self.csv_file)
#         self.csv_writer.writerow(['Timestamp', 'mic1_pos', 'mic2_pos', 'tello_pos', 'Distance1', 'Distance2', '1 Sound Level (dB)', '2 Sound Level (dB)'])
#         #self.csv_writer.writerow(['Timestamp', 'mic_pos', 'tello_pos', 'Distance', 'Sound Level (dB)']) #collect time, raw mic pos., raw drone pos, distance between tello and mic, sound level(dB)

#     def timer_callback(self):
#         #print("Mic2: ",self.mic_position2)
#         if self.tello_position is not None and self.mic_position is not None and self.mic_position2 is not None:
#             distance1 = np.linalg.norm(self.tello_position - self.mic_position)
#             #print("Mic2: ",self.mic_position2)
#             distance2 = np.linalg.norm(self.tello_position - self.mic_position2)
#             audio_data_1 = sd.rec(int(self.duration * 44100), samplerate=44100, channels=1, dtype='float32', device = device1)
#             #audio_data_2 = sd.rec(int(self.duration * 44100), samplerate=44100, channels=1, dtype='float32', device = device1)
#             sd.wait()
#             rms_amplitude_1 = np.sqrt(np.mean(np.square(audio_data_1)))
#             #rms_amplitude_2 = np.sqrt(np.mean(np.square(audio_data_2)))
#             db_1 = 20 * np.log10(rms_amplitude_1 / self.reference_amplitude)
#             #db_2 = 20 * np.log10(rms_amplitude_2 / self.reference_amplitude)
#             msg = Float32MultiArray()
#             msg.data = [float(db_1)]#, float(db_2)]
#             self.publisher_.publish(msg)
#             self.csv_writer.writerow([self.get_clock().now().to_msg(), np.round(self.mic_position, decimals = 2), np.round(self.mic_position2, decimals = 2), np.round(self.tello_position, decimals = 2), f"{distance1:.2f}", f"{distance2:.2f}", f"{db_1:.2f}", f"{2:.2f}"])
#             self.get_logger().info(f'Published Distance 1: {distance1:.2f} meters, Published Distance 2: {distance2:.2f} meters,  Sound Level 1: {db_1:.4f} dB, Sound Level 2: {1:.4f} dB')

#             # audio_data = sd.rec(int(self.duration * 44100), samplerate=44100, channels=1, dtype='float32')
#             # sd.wait()
#             # rms_amplitude = np.sqrt(np.mean(np.square(audio_data)))
#             # db = 20 * np.log10(rms_amplitude / self.reference_amplitude)
#             # self.publisher_.publish(Float32(data=db))
#             # self.csv_writer.writerow([self.get_clock().now().to_msg(), np.round(self.mic_position, decimals = 2), np.round(self.tello_position, decimals = 2), f"{distance:.2f}", f"{db:.2f}"])
#             # self.get_logger().info(f'Published Distance: {distance:.2f} meters, Sound Level: {db:.2f} dB')

#     def tello_pose_callback(self, msg):
#         self.tello_position = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])

#     def mic_pose_callback(self, msg):
#         self.mic_position = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
#     def mic_pose_callback2(self, msg):
#         self.mic_position2 = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])

#     def on_shutdown(self):
#         self.csv_file.close()

# def main(args=None):
#     rclpy.init(args=args)
#     '''parser = argparse.ArgumentParser(description='Sound and Distance Publisher Node')
#     parser.add_argument('--exp_name', type=str, default='test',help='The name of the drone data csv file')
#     args = parser.parse_args(args=rclpy.utilities.remove_ros_args(args)) #remove ROS-specific arguements'''
    
#     node = SoundAndDistancePublisher()
#     rclpy.spin(node)
#     node.on_shutdown()
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
