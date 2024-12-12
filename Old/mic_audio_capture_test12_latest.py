#!/usr/bin/env python3
import sounddevice as sd
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray 
# Capture and publish audio data

class AudioPublisher(Node):
    def __init__(self, device_indices, sample_rate = 44100, duration = 1):
        super().__init__('audio_publisher')
        self.audio_data_list=[0.] * 12 # List of length 12 containing the data for each mic
        self.audio_streams=[None] * 12 # List of length 12 containing the InputStream for each device

        for idx_device, device in enumerate(self.device_indices):
           self.audio_streams[idx_device] = sd.InputStream(
                device=args.device, channels=max(args.channels),
                samplerate=args.samplerate, 
                callback=lambda indata, frames, time, status : self.audio_callback(idx_device, indata, frames, time, status))
           # self.publishers_list.append(publisher)
        
        self.publisher=self.create_publisher(Float32MultiArray,f'audio_device{device}',10)
        self.timer = self.create_timer(duration, self.publish_data)
       
    def audio_callback(idx_device, indata, frames, time, status):
        # processing to obtain result
        self.audio_data_list[idx_device]=result

    def publish_data():
        #publishing audio chunk
        msg = Float32MultiArray()
        # self.get_logger().info(f"{audio_data.shape}")
        msg.data = audio_data.flatten().tolist()
        publisher.publish(msg)
        # self.get_logger().info(f"Message = {len(msg.data)}")
        self.get_logger().info(f"Published audio chunk to {deviceindex}")

class AudioPublisher(Node):
    def __init__(self, device_indices, sample_rate = 44100, duration = 1):
        super().__init__('audio_publisher')
        self.publishers_list = []
        self.device_indices = device_indices
        self.sample_rate = sample_rate
        self.duration = duration
        for device in self.device_indices:
            publisher = self.create_publisher(Float32MultiArray,f'audio_device{device}',10)
            # self.publishers_list.append(publisher)
            self.timer = self.create_timer(duration, lambda dev=device, pub=publisher: self.capture_and_publish(dev, pub))

    
    def capture_and_publish(self,deviceindex,publisher):
        self.get_logger().info(f"Recording on microphone {deviceindex}")
        audio_data = sd.rec(int(self.duration * self.sample_rate), samplerate=self.sample_rate, channels=1, dtype='float32',
            device=deviceindex)
        sd.wait() 
        self.get_logger().info("audio chunk recording complete")
        #publishing audio chunk
        msg = Float32MultiArray()
        # self.get_logger().info(f"{audio_data.shape}")
        msg.data = audio_data.flatten().tolist()
        publisher.publish(msg)
        # self.get_logger().info(f"Message = {len(msg.data)}")
        self.get_logger().info(f"Published audio chunk to {deviceindex}")


def main():
    # print(sd.query_devices())
    rclpy.init(args=None)
    device_indices = [0,1,11,12,13,14,15,16,17,18,19,20]
    audio_publisher = AudioPublisher(device_indices)
    try:
        rclpy.spin(audio_publisher)
    except KeyboardInterrupt:
        audio_publisher.get_logger().info("Shutting down publishers.")
    finally:
        audio_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()