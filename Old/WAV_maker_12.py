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

#WAV file creater from recorder_sender_12
class AudioSubscriber(Node):
    def __init__(self, sample_rate = 44100, duration = 1, number_of_mics = 12):
        super().__init__('audio_subscriber')

        self.sample_rate = sample_rate
        self.number_of_mics = number_of_mics
        self.chunks_list = [None] * self.number_of_mics
        self.samples_per_mic =  sample_rate*duration
  

        self.output_filename = "audiocaptures/audiocaptures_12/output_audio"

        self.wav_subscriber = self.create_subscription(Float32MultiArray, 'audio_data', self.audio_recieved_callback,10)

    def audio_recieved_callback(self, msg):
        # for incoming data, this callback separates the 12 mics' samples 
        # and concatenates them to their respective index in self.chunks_list
        flatlist = msg.data
        if len(flatlist) != self.samples_per_mic * self.number_of_mics:
            self.get_logger().error(f"Incoming audio data length mismatch! length of flatlist:{len(flatlist)} not equal to {self.samples_per_mic * self.number_of_mics}")
        for ind in range(self.number_of_mics):
            audio_chunk = np.array(flatlist[self.samples_per_mic*ind:self.samples_per_mic*(ind+1)], dtype=np.float32)
            if self.chunks_list[ind] is not None:
                self.chunks_list[ind] = np.concatenate((self.chunks_list[ind], audio_chunk.flatten()))
            else:
                self.chunks_list[ind] = audio_chunk.flatten()
            self.get_logger().info("Received audio chunk!!! yay")


    def save_audio(self):
        for device_index, audio in enumerate(self.chunks_list):
            if audio is None or audio.size == 0:
                self.get_logger().info(f"No audio received from mic {device_index}")
                continue
            try:
                currentdateandtime = datetime.datetime.now()
                output_file = f"{self.output_filename}_{currentdateandtime}_{device_index}.wav"
                write(output_file, self.sample_rate, audio)
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
        audio_subscriber.save_audio()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
 

