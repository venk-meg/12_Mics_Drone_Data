#!/usr/bin/env python3
import sounddevice as sd
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray 
import numpy as np
from threading import Lock

# Capture and publish audio data to csv_logger, sound

class AudioPublisher(Node):
    def __init__(self, device_indices, sample_rate = 44100, duration = 1, number_of_mics = 12):
        super().__init__('audio_publisher')
        self.device_indices = device_indices
        self.number_of_mics = number_of_mics
        self.reference_amplitude = 1 #change as per requirements
        #threading locks
        self.audio_data_lock = Lock()
        self.db_data_lock = Lock()

        # create publishers
        self.publish_sound = self.create_publisher(Float32MultiArray, 'audio_data', 10)
        self.publish_db = self.create_publisher(Float32MultiArray, 'db_data',10)

        self.db_data_list = [None] * number_of_mics # List of length 12 containing the data for each mic FOR DB EVERY .1s
        self.audio_data_list=[None] * number_of_mics # List of length 12 containing the data for each mic FOR AUDIO EVERY DURATION
        self.audio_streams=[None] * number_of_mics # List of length 12 containing the InputStream for each device
        #create inputstreams for each device
        for idx_device, device in enumerate(self.device_indices):
           self.audio_streams[idx_device] = sd.InputStream(device=device, channels=1,samplerate=sample_rate, 
                callback=lambda indata, frames, time, status : self.audio_callback(idx_device, indata, frames, 
                                                                                   time, status))
        # Start audio streams
        for stream in self.audio_streams:
            stream.start()

        #create timers
        self.timer = self.create_timer(duration, self.publish_audio_data)
        self.timer = self.create_timer(.1, self.publish_db_data) #publishes db data every .1 seconds
        
    def audio_callback(self, idx_device, indata, frames, time, status):
        #indata size is (frames = no. of samples, channels)
        # update indata to audio data list
        with self.audio_data_lock:
            if self.audio_data_list[idx_device] is not None:
                self.audio_data_list[idx_device]= np.vstack((self.audio_data_list[idx_device], indata))
            else:
                self.audio_data_list[idx_device] = indata
                # self.audio_data_list[idx_device] = np.vstack(([[self.number_identifier]], indata))
        # update indata to db data list
        with self.db_data_lock:
            if self.db_data_list[idx_device] is not None:
                self.db_data_list[idx_device]= np.vstack((self.db_data_list[idx_device], indata))
            else:
                self.db_data_list[idx_device] = indata

    def publish_audio_data(self):
        #flatten audio list data
        with self.audio_data_lock:
            # self.get_logger().info(f"SHAPE:{[audiothing.shape for audiothing in self.audio_data_list]}")
            flattened_audio_data_list = [
                float(value) for sublist in self.audio_data_list if sublist is not None for value in sublist.flatten()
            ]
            self.audio_data_list = [None] * self.number_of_mics
        #create message
        msg = Float32MultiArray()
        msg.data = flattened_audio_data_list
        #publish message
        self.publish_sound.publish(msg)
        # self.get_logger().info(f"Message = {len(msg.data)}")
        self.get_logger().info(f"Published audio chunks: {msg.data}")

    # def publish_audio_data(self):
    #     #flatten audio list data
    #     with self.audio_data_lock:
    #         msg = Float32MultiArray()
    #         msg.data = []
    #         for audiodata in self.audio_data_list:
    #             flat_audio_data = audiodata.flatten()
    #             msg.data.append(flat_audio_data)
             
    #         self.audio_data_list = [None] * self.number_of_mics

    #     #publish message
    #     self.publish_sound.publish(msg)
    #     # self.get_logger().info(f"Message = {len(msg.data)}")
    #     self.get_logger().info("Published audio chunks")
    
    def publish_db_data(self):
        #make db data  list and empty db_data_list
        with self.db_data_lock:
            db_list = []
            for list_item in self.db_data_list:
                if list_item is not None:
                    rms_amplitude = np.sqrt(np.mean(np.square(list_item)))
                    db = 20 * np.log10(rms_amplitude / self.reference_amplitude)
                    db_list.append(db)
                else:
                    db_list.append(0.0)
            self.db_data_list = [None] * self.number_of_mics
        #create message
        msgg = Float32MultiArray()
        msgg.data = db_list
        #publish message
        self.publish_db.publish(msgg)
        # self.get_logger().info(f"Message = {len(msg.data)}")
        self.get_logger().info(f"Published db data: {msgg.data}")

    def __del__(self):
        for stream in self.audio_streams:
            stream.stop()
            stream.close()

def main():
    rclpy.init(args=None)
    device_indices = [0, 1, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20]
    audio_publisher = AudioPublisher(device_indices)
    try:
        rclpy.spin(audio_publisher)
    except KeyboardInterrupt:
        audio_publisher.get_logger().info("Shutting down publishers.")
    finally:
        audio_publisher.destroy_node()
        del audio_publisher
        rclpy.shutdown()

if __name__ == "__main__":
    main()

# sd.query_devices()






# class AudioPublisher(Node):
#     def __init__(self, device_indices, sample_rate = 44100, duration = 1, number_of_mics = 12, number_identifier = 1010101.1010101):
#         super().__init__('audio_publisher')
#         self.device_indices = device_indices
#         self.number_of_mics = number_of_mics
#         self.reference_amplitude = 1 #change as per requirements
#         self.number_identifier = number_identifier
#         #threading locks
#         self.audio_data_lock = Lock()
#         self.db_data_lock = Lock()

#         # create publishers
#         self.publish_sound = self.create_publisher(Float32MultiArray, 'audio_data', 10)
#         self.publish_db = self.create_publisher(Float32MultiArray, 'db_data',10)

#         self.db_data_list = [None] * number_of_mics # List of length 12 containing the data for each mic FOR DB EVERY .1s
#         self.audio_data_list=[None] * number_of_mics # List of length 12 containing the data for each mic FOR AUDIO EVERY DURATION
#         self.audio_streams=[None] * number_of_mics # List of length 12 containing the InputStream for each device
#         #create inputstreams for each device
#         for idx_device, device in enumerate(self.device_indices):
#            self.audio_streams[idx_device] = sd.InputStream(device=device, channels=1,samplerate=sample_rate, 
#                 callback=lambda indata, frames, time, status : self.audio_callback(idx_device, indata, frames, 
#                                                                                    time, status))
#         # Start audio streams
#         for stream in self.audio_streams:
#             stream.start()

#         #create timers
#         self.timer = self.create_timer(duration, self.publish_audio_data)
#         self.timer = self.create_timer(0.1, self.publish_db_data) #publishes db data every 0.1 seconds
        
#     def audio_callback(self, idx_device, indata, frames, time, status):
#         #indata size is (frames = no. of samples, channels)
#         # update indata to audio data list
#         with self.audio_data_lock:
#             if self.audio_data_list[idx_device] is not None:
#                 self.audio_data_list[idx_device]= np.vstack((self.audio_data_list[idx_device], indata))
#             else:
#                 self.audio_data_list[idx_device] = indata
#                 # self.audio_data_list[idx_device] = np.vstack(([[self.number_identifier]], indata))
#         # update indata to db data list
#         with self.db_data_lock:
#             if self.db_data_list[idx_device] is not None:
#                 self.db_data_list[idx_device]= np.vstack((self.db_data_list[idx_device], indata))
#             else:
#                 self.db_data_list[idx_device] = indata

#     def publish_audio_data(self):
#         #flatten audio list data
#         with self.audio_data_lock:
#         #     flattened_audio_data_list = [
#         #     float(value) for sublist in self.audio_data_list if sublist is not None for value in sublist.flatten()
#         # ]
#             flattened_audio_data_list = [
#                 float(value) for sublist in self.audio_data_list if sublist is not None for value in sublist.flatten()
#             ]
#             self.audio_data_list = [None] * self.number_of_mics
#         #create message
#         msg = Float32MultiArray()
#         msg.data = flattened_audio_data_list
#         #publish message
#         self.publish_sound.publish(msg)
#         # self.get_logger().info(f"Message = {len(msg.data)}")
#         self.get_logger().info("Published audio chunks")
    
#     def publish_db_data(self):
#         #make db data  list and empty db_data_list
#         with self.db_data_lock:
#             db_list = []
#             for list_item in self.db_data_list:
#                 if list_item is not None:
#                     rms_amplitude = np.sqrt(np.mean(np.square(list_item)))
#                     db = 20 * np.log10(rms_amplitude / self.reference_amplitude)
#                     db_list.append(db)
#                 else:
#                     db_list.append(0.0)
#             self.db_data_list = [None] * self.number_of_mics
#         #create message
#         msg = Float32MultiArray()
#         msg.data = db_list
#         #publish message
#         self.publish_db.publish(msg)
#         # self.get_logger().info(f"Message = {len(msg.data)}")
#         self.get_logger().info("Published db data")

#     def __del__(self):
#         for stream in self.audio_streams:
#             stream.stop()
#             stream.close()

# def main():
#     rclpy.init(args=None)
#     device_indices = [0, 1, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20]
#     audio_publisher = AudioPublisher(device_indices)
#     try:
#         rclpy.spin(audio_publisher)
#     except KeyboardInterrupt:
#         audio_publisher.get_logger().info("Shutting down publishers.")
#     finally:
#         audio_publisher.destroy_node()
#         del audio_publisher
#         rclpy.shutdown()

# if __name__ == "__main__":
#     main()

# # sd.query_devices()