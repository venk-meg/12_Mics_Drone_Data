#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
import sounddevice as sd
from scipy.io.wavfile import write
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray 
from functools import partial
import datetime

class AudioSubscriber(Node):
    def __init__(self):
        super().__init__('audio_subscriber')
        device_indices = [0,1,11,12,13,14,15,16,17,18,19,20]
        self.audio_subscribers_list = []
        self.audio_buffers = {ind: [] for ind in device_indices}
        self.sample_rate = 44100

        for device_index in device_indices:
            subscription = self.create_subscription(Float32MultiArray, f'audio_device{device_index}', partial(self.callback, device_index=device_index),10)
            self.audio_subscribers_list.append((device_index,subscription))
    
    def callback(self, msg, device_index):
        audio_chunk = np.array(msg.data, dtype=np.float32)
        self.audio_buffers[device_index].append(audio_chunk)
        self.get_logger().info(f"Received audio chunk from mic {device_index}")

    def save_audio(self, output_file_name):
        for device_index, audio_chunks in self.audio_buffers.items():
            if not audio_chunks:
                self.get_logger().info(f"No audio chunks received from mic {device_index}")
                continue
            try:
                concatenated_audio = np.concatenate(audio_chunks, axis=0)
                currentdateandtime = datetime.datetime.now()
                output_file = f"{output_file_name}_{device_index}_{currentdateandtime}.wav"
                write(output_file, self.sample_rate, concatenated_audio)

                # with wave.open(output_file, 'w') as wav_file:
                #     wav_file.setnchannels(1)  
                #     wav_file.setsampwidth(2)
                #     wav_file.setframerate(self.sample_rate)
                   
                #     wav_file.writeframes((concatenated_audio * 32767).clip(-32768, 32767).astype(np.int16).tobytes())
                self.get_logger().info(f"Saved audio to {output_file}")
            except Exception as e:
                self.get_logger().error(f"Failed to save audio for mic {device_index}: {e}")


def main():
    rclpy.init(args=None)
    audio_subscriber = AudioSubscriber()
    try:
        rclpy.spin(audio_subscriber)
    except KeyboardInterrupt:
        audio_subscriber.get_logger().info("Shutting down subscriber.")
    finally:
        audio_subscriber.save_audio("audiocaptures/output_audio")
        rclpy.shutdown()

if __name__ == "__main__":
    main()
 