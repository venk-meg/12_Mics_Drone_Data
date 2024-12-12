# #!/usr/bin/env python3
# import sounddevice as sd
# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Float32
# import numpy as np
# from threading import Lock
# # from scipy.io.wavfile import write
# from scipy.io import wavfile
# import datetime
# import wave

# import os
# # import matplotlib.pyplot as plt
# # Capture and publish audio data to csv_logger, sound

# class AudioPublisher(Node):
#     def __init__(self, device_indices, sample_rate = 44100, duration = .1):
#         super().__init__('audio_publisher')

#         # Declare and retrieve the device_number parameter
#         self.declare_parameter('device_number')
#         self.device_number = self.get_parameter('device_number').value
#         if self.device_number is None or self.device_number >= len(device_indices):
#             self.get_logger().error(f"Invalid device_number: {self.device_number}")
#             raise ValueError(f"Invalid device_number: {self.device_number}")
#         self.device = device_indices[self.device_number]
#         self.get_logger().info(f"Initialized with device_number: {self.device_number}")
#         self.reference_amplitude = 1 #change as per requirements
#         self.sample_rate = sample_rate
#         self.output_filename = "audiocaptures/audiocaptures_12/output_audio"
#         self.buffer_size =  self.sample_rate*10 #empty buffer every 10 seconds
#         self.buffer_count = 0

#         #threading locks
#         self.audio_data_lock = Lock()
#         self.db_data_lock = Lock()

#         # create publisher
#         self.publish_db = self.create_publisher(Float32, f'db_data_{self.device_number}',10)

#         self.audio_data_list = []
#         self.db_data_list = None
#         self.audio_stream= sd.InputStream(device=self.device, channels=1,
#                                           samplerate=self.sample_rate, callback=self.audio_callback)
#         self.audio_stream.start()
#         self.timer = self.create_timer(duration, self.publish_db_data)
        

#     def audio_callback(self, indata, frames, time, status):
#         #indata size is (frames = no. of samples, channels)
#         # update indata to audio data list
#         #debug
#         #self.get_logger().info(f"Received {len(indata)} samples of audio data.")

#         with self.audio_data_lock:
#             self.audio_data_list.append(indata)
#         # update indata to db data list
#         with self.db_data_lock:
#             if self.db_data_list is not None:
#                 self.db_data_list= np.vstack((self.db_data_list, indata))
#             else:
#                 self.db_data_list = indata
#         #debug
#         # if len(self.audio_data_list) > 0:
#         #     self.get_logger().info(f"Buffer size: {len(self.audio_data_list)}")
#         # else:
#         #     self.get_logger().info("Buffer is empty.")
    
#         if len(self.audio_data_list)*len(indata) > self.buffer_size:
#             self.emptybuffer()

#     def publish_db_data(self):
#         #make db data  list and empty db_data_list
#         with self.db_data_lock:
#             if self.db_data_list is not None:
#                 rms_amplitude = np.sqrt(np.mean(np.square(self.db_data_list)))
#                 db = 20 * np.log10(rms_amplitude / self.reference_amplitude)
#             else:
#                 db = 0.0
#             self.db_data_list = None
#         #create message
#         msg = Float32()
#         msg.data = db
#         #publish message
#         self.publish_db.publish(msg)
#         # self.get_logger().info(f"Message = {len(msg.data)}")
#         # self.get_logger().info(f"Published db data: {msg.data}")

#     def emptybuffer(self):
#         # with self.audio_data_lock:
#         if len(self.audio_data_list) == 0:
#             self.get_logger().info(f"No audio received from mic {self.device_number}")
#             return
#         try:
#             audio_data = np.concatenate(self.audio_data_list, axis=0)
#             # audio_data = np.vstack(self.audio_data_list)
#             self.get_logger().info(f"SHAPE!! {audio_data.shape}")
#             #debug
#             # self.get_logger().info(f"Saving {len(audio_data)} samples to temp file.")
#             # self.get_logger().info(f"Audio min: {audio_data.min()}, max: {audio_data.max()}")

#             output_file = f"{self.output_filename}_TEMP_{self.buffer_count}_{self.device_number}.wav"
#             wavfile.write(output_file, self.sample_rate, audio_data.astype(np.float32))
            
#             self.audio_data_list = []
#             self.buffer_count += 1
#         except Exception as e:
#             self.get_logger().error(f"Failed to save TEMP audio for mic {self.device_number}: {e}")


#     # def emptybuffer(self):
#     #     if len(self.audio_data_list) == 0:
#     #         self.get_logger().info(f"No audio received from mic {self.device_number}")
#     #         return
#     #     try:
#     #         audio_data = np.vstack(self.audio_data_list)
#     #         self.get_logger().info(f"Audio min: {audio_data.min()}, max: {audio_data.max()}")
#     #         output_file = f"{self.output_filename}_TEMP_{self.buffer_count}_{self.device_number}.wav"
#     #         # Convert float32 to int16 as is in a standard wav file
#     #         audio_data_int16 = np.int16(audio_data * 32767)  # Assuming data is in range -1.0 to 1.0
            
#     #         # Create the wave file and write the data
#     #         with wave.open(output_file, 'wb') as f_out:
#     #             f_out.setnchannels(1)  # Mono audio (1 channel)
#     #             f_out.setsampwidth(2)  # 16-bit audio (2 bytes per sample)
#     #             f_out.setframerate(self.sample_rate)  # Set the sample rate (e.g., 44100 Hz)
#     #             f_out.writeframes(audio_data_int16.tobytes())  # Write audio frames
                
#     #         # Clear the audio data buffer and increment the buffer count
#     #         self.audio_data_list = []
#     #         self.buffer_count += 1
            
#         # except Exception as e:
#         #     self.get_logger().error(f"Failed to save TEMP audio for mic {self.device_number}: {e}")


#     def savewav(self):
#         self.emptybuffer()
#         try:
#             currentdateandtime = datetime.datetime.now()
#             output_file = f"{self.output_filename}_{currentdateandtime}_{self.device_number}.wav"
#             #concatenate and save temp files
#             temp_files = [f"{self.output_filename}_TEMP_{i}_{self.device_number}.wav" for i in range(self.buffer_count)]
#             self.concatenate_and_save_wav(temp_files, output_file)
#             self.get_logger().info(f"Successfully saved audio to {output_file}")
#             self.delete_temp_files(temp_files)
#         except Exception as e:
#             self.get_logger().error(f"Failed to save audio for mic {self.device_number}: {e}")

#     def concatenate_and_save_wav(self, temp_files, output_file):
#         all_audio_data = []
#         for temp_file in temp_files:
#             _, audio_data = wavfile.read(temp_file)
#             all_audio_data.append(audio_data)
#         concatenated_audio_data = np.concatenate(all_audio_data, axis=0)
#         wavfile.write(output_file, self.sample_rate, concatenated_audio_data)
#         # self.get_logger().info(f"Saved concatenated audio to {output_file}")

#     # def concatenate_and_save_wav(self, temp_files, output_file):
#     #     with wave.open(temp_files[0], 'rb') as first_file:
#     #         params = first_file.getparams()
#     #         with wave.open(output_file, 'wb') as f_out:
#     #             f_out.setparams(params)

#     #             for temp_file in temp_files:
#     #                 with wave.open(temp_file, 'rb') as f_in:
#     #                     frames = f_in.readframes(f_in.getnframes())
#     #                     f_out.writeframes(frames)

#     def delete_temp_files(self, temp_files):
#         for file in temp_files:
#             if os.path.exists(file):
#                 os.remove(file)
#                 self.get_logger().info(f"Deleted temporary file: {file}")
        
            
#     def __del__(self):
#         with self.audio_data_lock:
#             self.savewav()
#         # self.get_logger().info(f"Audio stream stopped for device {self.device_number}") 
#         self.audio_stream.stop()
#         self.audio_stream.close()

# def main():
#     rclpy.init(args=None)
#     device_indices = [0, 1, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20]
#     # audio_publisher = AudioPublisher(device_indices[device_number], device_number)
#     audio_publisher = AudioPublisher(device_indices)
#     try:
#         rclpy.spin(audio_publisher)
#     except KeyboardInterrupt:
#         audio_publisher.get_logger().info("Shutting down publisher.")
#     finally:
#         audio_publisher.destroy_node()
#         del audio_publisher
#         rclpy.shutdown()

# if __name__ == "__main__":
#     main()

# # sd.query_devices()





















#OG code



#!/usr/bin/env python3
import sounddevice as sd
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import numpy as np
from threading import Lock
from scipy.io.wavfile import write
import datetime
import matplotlib.pyplot as plt

# Capture and publish audio data to csv_logger, sound

class AudioPublisher(Node):
    def __init__(self, sample_rate = 44100, duration = 1):
        super().__init__('audio_publisher')

        # Declare and retrieve the device_number parameter
        self.declare_parameter('device_number')
        self.declare_parameter('device_indices')
        self.device_number = self.get_parameter('device_number').value
        device_indices = self.get_parameter('device_indices').value

        self.get_logger().info(f"INDICEx: {device_indices[self.device_number]}")
        self.get_logger().info(f"DEVICE NUMBER GIVEN: {self.device_number}")

        if self.device_number is None or self.device_number > len(device_indices):
            self.get_logger().error(f"Invalid device_port: {device_indices[self.device_number]}")
            raise ValueError(f"Invalid device_number: {self.device_number} & device_port = {device_indices[self.device_number]}")
        self.device_port = device_indices[self.device_number]

        self.reference_amplitude = 1 #change as per requirements
        self.sample_rate = sample_rate
        self.output_filename = "audiocaptures/audiocaptures_12/output_audio"

        #threading locks
        self.audio_data_lock = Lock()
        self.db_data_lock = Lock()

        # create publishers
        self.publish_db = self.create_publisher(Float32, f'db_data_{self.device_number}',10)

        self.audio_data_list = None
        self.db_data_list = None

        #checking fro channels errors
        # device_name_list = []
        # device_idx_list = []
        # for idx, device in enumerate(sd.query_devices()):
        #     if 'Lavalier' in device['name']:
        #         device_name_list.append(device)
        #         device_idx_list.append(idx)
        # self.get_logger().info(f"{self.device_number}: device_name_list len: {len(device_name_list)}")
        # self.get_logger().info(f"{self.device_number}: device_name_list: {device_idx_list}")
        # device_max_input_channels=device_name_list[self.device_number]['max_input_channels']
        # if device_max_input_channels != 1:
        #     device_name=device_name_list[self.device_number]['name']
        #     error_str=f"Device {device_name} has {device_max_input_channels} input channels"
        #     self.get_logger().error(error_str)
        #     raise ValueError(error_str)     

        try:            
            self.audio_stream= sd.InputStream(device=self.device_port, channels=1,
                                          samplerate=self.sample_rate, callback=self.audio_callback)
        except sd.PortAudioError as ex:
            print(ex)
            # print("pls")
            raise
        self.audio_stream.start()
        self.timer = self.create_timer(duration, self.publish_db_data)
        
    def audio_callback(self, indata, frames, time, status):
        #indata size is (frames = no. of samples, channels)
        # update indata to audio data list
        with self.audio_data_lock:
            if self.audio_data_list is not None:
                self.audio_data_list= np.vstack((self.audio_data_list, indata))
            else:
                self.audio_data_list = indata
        # update indata to db data list
        with self.db_data_lock:
            if self.db_data_list is not None:
                self.db_data_list= np.vstack((self.db_data_list, indata))
            else:
                self.db_data_list = indata

    def publish_db_data(self):
        #make db data  list and empty db_data_list
        with self.db_data_lock:
            if self.db_data_list is not None:
                rms_amplitude = np.sqrt(np.mean(np.square(self.db_data_list)))
                db = 20 * np.log10(rms_amplitude / self.reference_amplitude)
            else:
                db = 0.0
            self.db_data_list = None
        #create message
        msg = Float32()
        msg.data = db
        #publish message
        self.publish_db.publish(msg)
        # self.get_logger().info(f"db msg published")
        # self.get_logger().info(f"Published db data: {msg.data}")

    def savewav(self):
        # with self.audio_data_lock:
        if self.audio_data_list is None or self.audio_data_list.size == 0:
            self.get_logger().info(f"No audio received from mic {self.device_number}")
            return
        try:
            currentdateandtime = datetime.datetime.now()
            output_file = f"{self.output_filename}_{currentdateandtime}_{self.device_number}.wav"
            write(output_file, self.sample_rate, self.audio_data_list.astype(np.float32))
            self.get_logger().info(f"Saved audio to {output_file}")
            # self.audio_data_list = None
        except Exception as e:
            self.get_logger().error(f"Failed to save audio for mic {self.device_number}: {e}")

    def __del__(self):
        with self.audio_data_lock:
            self.savewav()
        self.get_logger().info(f"Audio stream stopped for device {self.device_number}") 
        self.audio_stream.stop()
        self.audio_stream.close()


def main():
    rclpy.init(args=None)
    audio_publisher = AudioPublisher()
    try:
        rclpy.spin(audio_publisher)
    except KeyboardInterrupt:
        audio_publisher.get_logger().info("Shutting down publisher.")
    finally:
        audio_publisher.destroy_node()
        del audio_publisher
        rclpy.shutdown()

if __name__ == "__main__":
    main()

# sd.query_devices()