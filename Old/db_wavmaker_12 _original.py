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
    def __init__(self, device_indices, sample_rate = 44100, duration = .1):
        super().__init__('audio_publisher')

        # Declare and retrieve the device_number parameter
        self.declare_parameter('device_number')

        self.device_number = self.get_parameter('device_number').value

        if self.device_number is None or self.device_number >= len(device_indices):
            self.get_logger().error(f"Invalid device_number: {self.device_number}")
            raise ValueError(f"Invalid device_number: {self.device_number}")
        
        self.device = device_indices[self.device_number]
        self.get_logger().info(f"Initialized with device_number: {self.device_number}")

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

        self.audio_stream= sd.InputStream(device=self.device, channels=1,
                                          samplerate=self.sample_rate, callback=self.audio_callback)
        
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
        # self.get_logger().info(f"Message = {len(msg.data)}")
        self.get_logger().info(f"Published db data: {msg.data}")

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
    device_indices = [0, 1, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20]
    # audio_publisher = AudioPublisher(device_indices[device_number], device_number)
    audio_publisher = AudioPublisher(device_indices)
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